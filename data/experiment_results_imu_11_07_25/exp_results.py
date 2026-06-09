"""
Purpose
-------
Estimate wheel slip from physical lemniscate (Gerono) trajectory experiments
conducted on 2025/07/11. Five runs of the same experiment are stored as separate
CSV pairs (cmd_vel + IMU). This script consolidates all five into one place.

Pipeline (applied to each experiment)
--------------------------------------
1. Load cmd_vel and IMU CSV data exported from ROS bags.
2. Clean columns, filter each recording to its active time window.
3. Convert IMU quaternions to Euler angles (extract yaw / theta).
4. Synchronize cmd_vel and IMU streams on a shared unix timestamp index.
5. Interpolate and unwrap theta to get a continuous heading signal.
6. Compute per-step heading change (theta_diff) as the measurement input to RLS.
7. Run Recursive Least Squares (RLS) to estimate wheel slip from the difference
   between commanded angular velocity and measured heading change.
8. Plot: theta unwrapped, estimated slip, RLS correction term, Kalman gain.

Usage
-----
Set SELECTED_EXPERIMENT to 1-5 to process a single run, or to None to run all.
"""

import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), "..", ".."))
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

from mathematical_simulator_class.robot import Robot
from mathematical_simulator_class.file_reader import Analysis
from mathematical_simulator_class.feedforward import Feedforward
from mathematical_simulator_class.recursive_least_square import RecursiveLeastSquares
from mathematical_simulator_class.compensator import Compensator
from mathematical_simulator_class.interpolation import InterpolationOfTime


# --- Experiment configuration ---
# drop_idx: row index of a known discontinuity in theta_unwrapped that must be
# removed before RLS estimation; caused by a momentary IMU dropout or sync gap.
EXPERIMENTS = [
    {
        "num":     1,
        "cmd":     "gerono-cmd_vel_exp1.csv",
        "imu":     "gerono-imu1_exp1.csv",
        "t_start": "2025/07/11/14:06:12.495318",
        "t_end":   "2025/07/11/14:06:32.141455",
        "t_op":    ">=",
        "drop_idx": 190,   # discontinuity at index 190 in synced data
    },
    {
        "num":     2,
        "cmd":     "gerono-cmd_vel_exp2.csv",
        "imu":     "gerono-imu1_exp2.csv",
        "t_start": "2025/07/11/14:07:38.258400",
        "t_end":   "2025/07/11/14:07:57.861537",
        "t_op":    ">=",
        "drop_idx": 196,   # discontinuity at index 196 in synced data
    },
    {
        "num":     3,
        "cmd":     "gerono-cmd_vel_exp3.csv",
        "imu":     "gerono-imu1_exp3.csv",
        "t_start": "2025/07/11/14:08:39.812057",
        "t_end":   "2025/07/11/14:08:59.533433",
        "t_op":    ">=",
        "drop_idx": 30,    # discontinuity at index 30 in synced data
    },
    {
        "num":     4,
        "cmd":     "gerono-cmd_vel_exp4.csv",
        "imu":     "gerono-imu1_exp4.csv",
        "t_start": "2025/07/11/14:09:36.853596",
        "t_end":   "2025/07/11/14:09:56.508609",
        "t_op":    ">",
        "drop_idx": 193,   # discontinuity at index 193 in synced data
    },
    {
        "num":     5,
        "cmd":     "gerono-cmd_vel_exp5.csv",
        "imu":     "gerono-imu1_exp5.csv",
        "t_start": "2025/07/11/14:10:39.520982",
        "t_end":   "2025/07/11/14:10:59.171122",
        "t_op":    ">",
        "drop_idx": 35,    # discontinuity at index 35 in synced data
    },
]

# Set to 1-5 to run a single experiment, or None to run all.
SELECTED_EXPERIMENT = 1


# --- Pipeline ---

def run_experiment(cfg):
    num = cfg["num"]
    print(f"\n{'='*50}")
    print(f"Running experiment {num}")
    print(f"{'='*50}")

    file_reader  = Analysis()
    interpolation = InterpolationOfTime()
    recursive    = RecursiveLeastSquares(s0=np.eye(1), P0=10 * np.eye(1), R=0.00436 * np.eye(1))
    compensator  = Compensator()

    data_dir = os.path.join(sys.path[0], 'data_exp')
    file_path     = os.path.join(data_dir, cfg["cmd"])
    file_path_imu = os.path.join(data_dir, cfg["imu"])

    # Load raw data
    df_cmd = file_reader.read_csv(file_path)
    df_imu = file_reader.read_csv(file_path_imu)

    # Clean columns
    df_cmd_filtered = file_reader.clean_data_cmd_vel(df_cmd)
    df_imu_filtered = file_reader.clean_data_imu(df_imu)

    # Convert quaternions to Euler angles
    df_imu_filtered = interpolation.quaternions_to_euler(df_imu_filtered)

    # Trim IMU to the active recording window
    if cfg["t_op"] == ">=":
        df_imu_filtered = df_imu_filtered[df_imu_filtered['time'] >= cfg["t_start"]]
    else:
        df_imu_filtered = df_imu_filtered[df_imu_filtered['time'] >  cfg["t_start"]]
    df_imu_filtered = df_imu_filtered[df_imu_filtered['time'] <= cfg["t_end"]]
    df_imu_filtered.reset_index(drop=True, inplace=True)

    # Keep only timesteps where the robot is turning
    df_cmd_filtered = df_cmd[df_cmd['.angular.z'] != 0]
    df_cmd_filtered.reset_index(drop=True, inplace=True)

    # Convert timestamps to unix and drop the original time column
    df_cmd_filtered = interpolation.convert_unix(df_cmd_filtered)
    df_imu_filtered = interpolation.convert_unix(df_imu_filtered)
    df_cmd_filtered.drop(columns=['time'], inplace=True)
    df_imu_filtered.drop(columns=['time'], inplace=True)

    # Align both streams on unix timestamp
    df_imu_filtered.set_index('unix_timestamp', inplace=True)
    df_cmd_filtered.set_index('unix_timestamp', inplace=True)
    df_cmd_filtered.rename(columns={'.angular.z': 'nominal_angular_velocity'}, inplace=True)
    df_cmd_filtered = df_cmd_filtered.drop(
        ['.linear.x', '.linear.y', '.linear.z', '.angular.x', '.angular.y'], axis=1
    )

    df_combined = pd.concat([df_cmd_filtered, df_imu_filtered], ignore_index=False, axis=1)
    df_combined.sort_index(inplace=True)
    print("Combined dataframe before interpolation:", df_combined.head(10))

    # Fill IMU gaps via interpolation and unwrap heading
    df_combined['theta'] = df_combined['theta'].interpolate(method='index', limit_direction='both')
    df_combined['theta_unwrapped'] = np.unwrap(df_combined['theta'].values)

    # Keep only rows where a cmd_vel command was issued
    df_cmd_synced = df_combined[df_combined['nominal_angular_velocity'].notna()]
    df_cmd_synced.reset_index(drop=True, inplace=True)
    print("cmd synced with imu:", df_cmd_synced)

    # Remove the known discontinuity row before computing heading differences
    df_cmd_synced = df_cmd_synced.drop(index=cfg["drop_idx"])
    df_cmd_synced.reset_index(drop=True, inplace=True)

    # Plot unwrapped heading
    plt.figure(figsize=(10, 5))
    plt.plot(df_cmd_synced['theta_unwrapped'], label='Theta Unwrapped', color='orange')
    plt.xlabel('Index')
    plt.ylabel('Theta Unwrapped (radians)')
    plt.title(f'Exp {num} — Theta Unwrapped Over Time')
    plt.legend()
    plt.grid()
    plt.show()

    # Heading change per step (measurement for RLS)
    df_cmd_synced['theta_diff'] = df_cmd_synced['theta_unwrapped'].diff().fillna(0)
    print("Theta difference for cmd synced data:", df_cmd_synced.head(20))

    # RLS slip estimation
    delta_t = 0.05
    slip   = []
    errors = []
    for timestep in np.arange(np.size(df_cmd_synced['theta_diff'])):
        C_matrix = np.array([df_cmd_synced['nominal_angular_velocity'][timestep] * delta_t])
        recursive.predict_exp(
            df_cmd_synced['nominal_angular_velocity'][timestep] * delta_t
            - df_cmd_synced['theta_diff'][timestep],
            C_matrix
        )
        slip.append(recursive.estimates[timestep][0])
        errors.append(recursive.errors[timestep][0])

    print(f"Exp {num} — final estimated slip: {slip[-1].item():.6f}")

    # Plot estimated slip
    plt.figure(figsize=(10, 4))
    plt.plot(slip, label='Estimated Slip')
    plt.xlabel('Time Step')
    plt.ylabel('Slip Value')
    plt.title(f'Exp {num} — Estimated Slip Over Time')
    plt.legend()
    plt.grid()
    plt.show()

    # Plot RLS correction term
    plt.figure(figsize=(10, 4))
    plt.plot(recursive.errors, label='Correction term')
    plt.xlabel('Time Step')
    plt.ylabel('Correction Value')
    plt.title(f'Exp {num} — Correction Term Over Time')
    plt.legend()
    plt.grid()
    plt.show()

    # Plot Kalman gain
    plt.figure(figsize=(10, 4))
    plt.plot(recursive.gainMatrices, label='Kalman Gain')
    plt.xlabel('Time Step')
    plt.ylabel('Kalman Gain Value')
    plt.title(f'Exp {num} — Kalman Gain Over Time')
    plt.legend()
    plt.grid()
    plt.show()


# --- Entry point ---

if SELECTED_EXPERIMENT is not None:
    cfg = next(e for e in EXPERIMENTS if e["num"] == SELECTED_EXPERIMENT)
    run_experiment(cfg)
else:
    for cfg in EXPERIMENTS:
        run_experiment(cfg)

