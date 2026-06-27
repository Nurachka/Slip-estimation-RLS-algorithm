# Lemniscate trajectory tracking with LinearMPC and online RLS slip estimation — sinusoidal slip.
#
# Robot runs with optional Gaussian sensor noise. The true wheel slip varies sinusoidally over time
# (same profile as feedforward_rls_sinusoidal_slip.py: oscillates 0.01-0.08 at 0.05 Hz, one full
# cycle every 400 timesteps at dt=0.05 s). To follow a changing slip the RLS uses a forgetting
# factor (lambda = 0.97) so it down-weights old measurements; the (unclipped) estimate is fed into
# the MPC model each step.
#
# Compares two scenarios under the same sinusoidal slip:
#   1. Feedforward only (no MPC) — uncompensated baseline
#   2. MPC + online RLS — MPC slip value updated each step from the RLS estimate
#
# Outputs: trajectory, position error, slip estimate vs true slip, heading, wheel velocity
# corrections, and a slip-error/position-error phase portrait. Also prints position/heading error
# and slip-estimation RMSE/MAE (full run and steady state) per scenario.

import sys
import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
sys.path.append("..")
from mathematical_simulator_class.linear_mpc import LinearMPC
from mathematical_simulator_class.recursive_least_square import RecursiveLeastSquares
from mathematical_simulator_class.config import NOISE_STD_POSITION, NOISE_STD_ORIENTATION

# --- Parameters ---
DT           = 0.05
WHEEL_BASE   = 0.5
N            = 10
VR_MAX       = 0.7
VL_MAX       = 0.7

# Sinusoidal slip profile (same as feedforward_rls_sinusoidal_slip.py)
SLIP_OFFSET  = 0.045         # center = (0.08 + 0.01) / 2
SLIP_AMP     = 0.035         # half-amplitude = (0.08 - 0.01) / 2
SLIP_FREQ_HZ = 0.05          # Hz — one full cycle every 400 timesteps at dt=0.05 s
LAMBDA       = 0.97          # RLS forgetting factor (< 1 so the estimate can track the sinusoid)

STEADY_STATE_START = 100
INNOVATION        = []
estimationerrorcovariancematrices = []

# --- Load trajectory ---
current_dir     = os.path.dirname(os.path.abspath(__file__))
trajectory_path = os.path.join(current_dir, '..', 'trajectories', 'lemniscate_trajectory.csv')
df        = pd.read_csv(trajectory_path)
x_ref     = df['x'].values
y_ref     = df['y'].values
theta_ref = df['theta'].values
vr_ref    = df['right_vel'].values
vl_ref    = df['left_vel'].values
n_steps   = len(df)
time      = df['time'].values


def slip_fn(k):
    return SLIP_OFFSET + SLIP_AMP * np.sin(2 * np.pi * SLIP_FREQ_HZ * k * DT)


def run_simulation(use_mpc=True, use_rls=False, s_mpc=0.0, use_noise=False, seed=42):
    np.random.seed(seed)

    mpc = LinearMPC(dt=DT, wheel_base=WHEEL_BASE, N_horizon=N,
                    vr_max=VR_MAX, vl_max=VL_MAX, s=s_mpc, du_max=None)

    rls        = RecursiveLeastSquares(s0=np.array([0.0]), P0=50.0 * np.eye(1), R=3.8e-5 * np.eye(1))
    theta_prev = theta_ref[0]

    x_a, y_a, theta_a = x_ref[0], y_ref[0], theta_ref[0]

    actual_states   = []
    delta_vr_list   = []
    delta_vl_list   = []
    slip_estimates  = []
    theta_meas_list = []

    for k in range(n_steps):

        # --- Robot moves first ---
        if k == 0:
            vr = vr_ref[k]
            vl = vl_ref[k]
        else:
            vr = vr_ref[k] + delta_vr
            vl = vl_ref[k] + delta_vl
        s_k     = slip_fn(k)
        v_a     = (1 - s_k) * (vr + vl) / 2.0
        omega_a = (1 - s_k) * (vr - vl) / WHEEL_BASE
        x_a     += v_a * np.cos(theta_a) * DT
        y_a     += v_a * np.sin(theta_a) * DT
        theta_a += omega_a * DT

        # --- Record post-movement state ---
        actual_states.append((x_a, y_a, theta_a))

        # --- Noisy measurements ---
        if use_noise:
            x_meas     = x_a     + np.random.normal(0, NOISE_STD_POSITION)
            y_meas     = y_a     + np.random.normal(0, NOISE_STD_POSITION)
            theta_meas = theta_a + np.random.normal(0, NOISE_STD_ORIENTATION)
        else:
            x_meas, y_meas, theta_meas = x_a, y_a, theta_a

        # --- RLS: uses vr/vl that just moved the robot ---
        if use_rls:

            rls.predict_sim_with_forgetting_factor(theta_meas, theta_prev, vr, vl, DT, lam=LAMBDA)
            s_hat = float(rls.estimates[-1][0])
            mpc.s = s_hat
            slip_estimates.append(s_hat)
            INNOVATION.append(rls.errors[-1][0])
            estimationerrorcovariancematrices.append(rls.estimationErrorCovarianceMatrices[-1][0])

        theta_meas_list.append(theta_meas)
        theta_prev = theta_meas

        # --- MPC: plans correction for next step ---
        if use_mpc:
            error_state = mpc.compute_error_state(
                np.array([x_meas, y_meas, theta_meas]),
                np.array([x_ref[min(k+1, n_steps - 1)], y_ref[min(k+1, n_steps - 1)], theta_ref[min(k+1, n_steps - 1)]])
            )
            A_list, B_list = [], []
            vr_ref_list, vl_ref_list = [], []
            for i in range(N):
                future_idx = min(k + 1 + i, n_steps - 1)
                A_i, B_i = mpc.define_AB_matrices(
                    theta_ref[future_idx], vr_ref[future_idx], vl_ref[future_idx]
                )
                A_list.append(A_i)
                B_list.append(B_i)
                vr_ref_list.append(vr_ref[future_idx])
                vl_ref_list.append(vl_ref[future_idx])
            delta_vr, delta_vl = mpc.solve(error_state, A_list, B_list, vr_ref_list, vl_ref_list)
        else:
            delta_vr, delta_vl = 0.0, 0.0

        delta_vr_list.append(delta_vr)
        delta_vl_list.append(delta_vl)

    return (np.array(actual_states),
            np.array(delta_vr_list),
            np.array(delta_vl_list),
            slip_estimates,
            np.array(theta_meas_list))


# --- Run scenarios ---
states_ff,  dvr_ff,  dvl_ff,  _,         _            = run_simulation(use_mpc=False, use_noise=True)
states_mpc, dvr_mpc, dvl_mpc, _,         _            = run_simulation(use_mpc=True,  use_rls=False, s_mpc=0.0, use_noise=True)
states_rls, dvr_rls, dvl_rls, slips_rls, theta_meas_rls = run_simulation(use_mpc=True,  use_rls=True, use_noise=True)

# --- True slip over the run (per timestep) ---
true_slip = np.array([slip_fn(k) for k in range(n_steps)])

# --- Figure 1: Trajectory ---
plt.figure(figsize=(7, 7))
plt.plot(x_ref, y_ref, 'r--', label='Reference', linewidth=1.5)
plt.plot(states_ff[:, 0],  states_ff[:, 1],  color='orange', label='Feedforward (no MPC)')
plt.plot(states_mpc[:, 0], states_mpc[:, 1], color='blue',   label='MPC slip-unaware (s=0)')
plt.plot(states_rls[:, 0], states_rls[:, 1], color='green',  label='MPC + online RLS')
plt.xlabel('X (m)')
plt.ylabel('Y (m)')
plt.title('Lemniscate Trajectory — MPC + Online RLS (Sinusoidal Slip)')
plt.legend()
plt.axis('equal')
plt.grid(True)
plt.tight_layout()
plt.show()

# --- Figure 2: Position tracking error ---
ref_xy    = np.column_stack((x_ref, y_ref))
error_ff  = np.linalg.norm(ref_xy - states_ff[:, :2],  axis=1)
error_mpc = np.linalg.norm(ref_xy - states_mpc[:, :2], axis=1)
error_rls = np.linalg.norm(ref_xy - states_rls[:, :2], axis=1)

plt.figure(figsize=(8, 4))
plt.plot(time, error_ff,  color='orange', label='Feedforward (no MPC)')
plt.plot(time, error_mpc, color='blue',   label='MPC slip-unaware (s=0)')
plt.plot(time, error_rls, color='green',  label='MPC + online RLS')
plt.xlabel('Time (s)')
plt.ylabel('Position error (m)')
plt.title('Position Tracking Error')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()

#Mean and max position error for each scenario
print(f'Feedforward      — position mean error: {error_ff.mean():.4f} m, max: {error_ff.max():.4f} m')
print(f'MPC slip-unaware — position mean error: {error_mpc.mean():.4f} m, max: {error_mpc.max():.4f} m')
print(f'MPC + online RLS — position mean error: {error_rls.mean():.4f} m, max: {error_rls.max():.4f} m')

# --- Heading error ---
heading_err_ff  = np.arctan2(np.sin(states_ff[:, 2]  - theta_ref), np.cos(states_ff[:, 2]  - theta_ref))
heading_err_mpc = np.arctan2(np.sin(states_mpc[:, 2] - theta_ref), np.cos(states_mpc[:, 2] - theta_ref))
heading_err_rls = np.arctan2(np.sin(states_rls[:, 2] - theta_ref), np.cos(states_rls[:, 2] - theta_ref))

plt.figure(figsize=(8, 4))
plt.plot(time, heading_err_ff,  color='orange', label='Feedforward (no MPC)')
plt.plot(time, heading_err_mpc, color='blue',   label='MPC slip-unaware (s=0)')
plt.plot(time, heading_err_rls, color='green',  label='MPC + online RLS')
plt.axhline(0, color='black', linestyle='--', linewidth=1.0)
plt.xlabel('Time (s)')
plt.ylabel('Heading error θ_actual − θ_ref (rad)')
plt.title('Heading Error Over Time')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()

print(f'Feedforward      — mean |θ err|: {np.abs(heading_err_ff).mean():.4f} rad, max: {np.abs(heading_err_ff).max():.4f} rad')
print(f'MPC slip-unaware — mean |θ err|: {np.abs(heading_err_mpc).mean():.4f} rad, max: {np.abs(heading_err_mpc).max():.4f} rad')
print(f'MPC + online RLS — heading mean |θ err|: {np.abs(heading_err_rls).mean():.4f} rad, max: {np.abs(heading_err_rls).max():.4f} rad')
# print rmse and std dev of position and heading errors for MPC + RLS scenario
print(f'MPC + online RLS — position RMSE: {np.sqrt(np.mean(error_rls**2)):.4f} m, heading RMSE: {np.sqrt(np.mean(heading_err_rls**2)):.4f} rad')
print(f'MPC + online RLS — position std dev: {error_rls.std():.4f} m, heading std dev: {np.degrees(heading_err_rls.std()):.4f} deg')

# Improvement of slip-aware RLS over slip-unaware MPC (positive = RLS better)
pos_impr  = (error_mpc.mean() - error_rls.mean()) / error_mpc.mean() * 100
head_impr = (np.abs(heading_err_mpc).mean() - np.abs(heading_err_rls).mean()) / np.abs(heading_err_mpc).mean() * 100
print(f'\nMPC+RLS vs MPC slip-unaware — position mean improvement: {pos_impr:+.2f}%, heading mean improvement: {head_impr:+.2f}%')


# RMSE and MAE of slip estimation (against the time-varying true slip)

slips_arr = np.array(slips_rls)
rmse_full = np.sqrt(np.mean((slips_arr - true_slip) ** 2))
mae_full  = np.mean(np.abs(slips_arr - true_slip))
slips_ss  = slips_arr[STEADY_STATE_START:]
true_ss   = true_slip[STEADY_STATE_START:]
rmse_ss   = np.sqrt(np.mean((slips_ss - true_ss) ** 2))
mae_ss    = np.mean(np.abs(slips_ss - true_ss))

print(f'\nSlip estimation (full run)         — RMSE: {rmse_full:.4f}, MAE: {mae_full:.4f}')
print(f'Slip estimation (steps {STEADY_STATE_START}→end) — RMSE: {rmse_ss:.4f}, MAE: {mae_ss:.4f}')

# --- Figure 3: Slip estimation ---
plt.figure(figsize=(8, 4))
plt.plot(time, true_slip, 'k--', linewidth=1.5, label='True slip')
plt.plot(time, slips_rls, color='green', label=f'RLS estimate (λ={LAMBDA})')
plt.xlabel('Time (s)')
plt.ylabel('Slip')
plt.title('Online RLS Slip Estimation — Sinusoidal Slip')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()

# --- Figure 4: Theta heading nominal vs measured ---
plt.figure(figsize=(8, 4))
plt.plot(time, theta_ref,       'r--',           label='Reference (nominal)',       linewidth=1.5)
plt.plot(time, states_ff[:, 2], color='orange',  label='Actual (feedforward)',      alpha=0.9)
plt.plot(time, theta_meas_rls,  color='green',   label='Measured (with noise, RLS)', alpha=0.8)
plt.xlabel('Time (s)')
plt.ylabel('Heading θ (rad)')
plt.title('Heading: Nominal vs Actual vs Measured')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()

# --- Figure 5: Reference vs commanded wheel velocities ---
fig, axes = plt.subplots(1, 2, figsize=(12, 4))
axes[0].plot(time, vr_ref,            'r--',         label='Reference',        linewidth=1.5)
axes[0].plot(time, vr_ref + dvr_mpc,  color='blue',  label='MPC slip-unaware (s=0)')
axes[0].plot(time, vr_ref + dvr_rls,  color='green', label='MPC + online RLS')
axes[0].set_xlabel('Time (s)')
axes[0].set_ylabel('Right Wheel Velocity (m/s)')
axes[0].set_title('Right Wheel Velocity: Reference vs Commanded')
axes[0].legend()
axes[0].grid(True)

axes[1].plot(time, vl_ref,            'r--',         label='Reference',        linewidth=1.5)
axes[1].plot(time, vl_ref + dvl_mpc,  color='blue',  label='MPC slip-unaware (s=0)')
axes[1].plot(time, vl_ref + dvl_rls,  color='green', label='MPC + online RLS')
axes[1].set_xlabel('Time (s)')
axes[1].set_ylabel('Left Wheel Velocity (m/s)')
axes[1].set_title('Left Wheel Velocity: Reference vs Commanded')
axes[1].legend()
axes[1].grid(True)
plt.tight_layout()
plt.show()

# --- Figure 6: Phase portrait — slip error vs position error (MPC + RLS only) ---
plt.figure(figsize=(6, 5))
sc = plt.scatter(slips_arr - true_slip, error_rls, c=time, cmap='viridis', s=10)
plt.colorbar(sc, label='Time (s)')
plt.axvline(0, color='black', linestyle='--', linewidth=1.0)
plt.xlabel('Slip error (s_hat − s_true)')
plt.ylabel('Position error (m)')
plt.title('Phase Portrait: Slip Error vs Position Error (MPC + RLS)')
plt.grid(True)
plt.tight_layout()
plt.show()

# # --- Figure 5: Innovation (error) over time ---
# plt.figure(figsize=(8, 4))
# plt.plot(time, INNOVATION, color='blue', label='Innovation (error)')
# plt.xlabel('Time (s)')
# plt.ylabel('Innovation')
# plt.title('Innovation (Error) Over Time')
# plt.legend()
# plt.grid(True)
# plt.tight_layout()
# plt.show()

# # # --- Figure 6: Estimation error covariance over time ---
# plt.figure(figsize=(8, 4))
# plt.plot(time, estimationerrorcovariancematrices, color='purple', label='Estimation Error Covariance')
# plt.xlabel('Time (s)')
# plt.ylabel('Estimation Error Covariance')
# plt.title('Estimation Error Covariance Over Time')
# plt.legend()
# plt.grid(True)
# plt.tight_layout()
# plt.show()
