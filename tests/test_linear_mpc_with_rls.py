# Lemniscate trajectory tracking with LinearMPC and online RLS slip estimation.
#
# Robot runs with optional Gaussian sensor noise. RLS estimates surface slip online
# from heading measurements; the estimate is fed into the MPC model each step.
# Slip estimate is clipped to [-0.2, 0.2] throughout to limit the impact of the
# first ~50 steps before RLS has had time to converge.
#
# Compares three scenarios with true slip s=0.1:
#   1. Feedforward only (no MPC) — uncompensated baseline
#   2. MPC slip-unaware — MPC active, s_mpc fixed at 0
#   3. MPC + online RLS — MPC slip value updated each step from RLS estimate
#
# Outputs: trajectory, position error, slip estimate over time, and wheel velocity
# corrections (4 figures). Also prints final and mean position error per scenario.

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
VR_MAX       = 1.0
VL_MAX       = 1.0
S_ACTUAL     = 0.2
LAMBDA       = 0.96
WARMUP_STEPS = 50
SLIP_CLIP    = 0.15

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


def run_simulation(s_actual=0.1, use_mpc=True, use_rls=False, s_mpc=0.0,
                   use_noise=False, seed=42):
    np.random.seed(seed)

    mpc = LinearMPC(dt=DT, wheel_base=WHEEL_BASE, N_horizon=N,
                    vr_max=VR_MAX, vl_max=VL_MAX, s=s_mpc, du_max=None)

    rls        = RecursiveLeastSquares(s0=np.array([0.1]), P0=10.0 * np.eye(1), R=0.00436 * np.eye(1))
    theta_prev = theta_ref[0]

    x_a, y_a, theta_a = x_ref[0], y_ref[0], theta_ref[0]

    actual_states  = []
    errors         = []
    delta_vr_list  = []
    delta_vl_list  = []
    slip_estimates = []

    for k in range(n_steps):

        # --- Robot moves first ---
        if k == 0:
            vr = vr_ref[k]
            vl = vl_ref[k]
        else:
            vr = vr_ref[k] + delta_vr
            vl = vl_ref[k] + delta_vl
        v_a     = (1 - s_actual) * (vr + vl) / 2.0
        omega_a = (1 - s_actual) * (vr - vl) / WHEEL_BASE
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
            if k == 0:
                rls.predict_sim(theta_meas, theta_prev, vr_ref[k], vl_ref[k], DT)
            else:
                rls.predict_sim(theta_meas, theta_prev, vr, vl, DT)
            s_hat = float(np.clip(rls.estimates[-1][0], -SLIP_CLIP, SLIP_CLIP))
            mpc.s = s_hat
            slip_estimates.append(rls.estimates[-1][0])

        theta_prev = theta_meas

        # --- MPC: plans correction for next step ---
        if use_mpc:
            error_state = mpc.compute_error_state(
                np.array([x_meas, y_meas, theta_meas]),
                np.array([x_ref[k], y_ref[k], theta_ref[k]])
            )
            A_list, B_list = [], []
            vr_ref_list, vl_ref_list = [], []
            for i in range(N):
                future_idx = min(k + i, n_steps - 1)
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
            np.array(errors),
            np.array(delta_vr_list),
            np.array(delta_vl_list),
            slip_estimates)


# --- Run scenarios ---
states_ff,      errors_ff,      dvr_ff,      dvl_ff,      _         = run_simulation(s_actual=S_ACTUAL, use_mpc=False, use_noise=True)
states_rls,     errors_rls,     dvr_rls,     dvl_rls,     slips_rls = run_simulation(s_actual=S_ACTUAL, use_mpc=True,  use_rls=True, use_noise=True)

# --- Figure 1: Trajectory ---
plt.figure(figsize=(7, 7))
plt.plot(x_ref, y_ref, 'r--', label='Reference', linewidth=1.5)
plt.plot(states_ff[:, 0],  states_ff[:, 1],  color='orange', label='Feedforward (no MPC)')
plt.plot(states_rls[:, 0], states_rls[:, 1], color='green',  label='MPC + online RLS')
plt.xlabel('X (m)')
plt.ylabel('Y (m)')
plt.title('Lemniscate Trajectory — MPC + Online RLS')
plt.legend()
plt.axis('equal')
plt.grid(True)
plt.tight_layout()
plt.show()

# --- Figure 2: Position tracking error ---
ref_xy    = np.column_stack((x_ref, y_ref))
error_ff  = np.linalg.norm(ref_xy - states_ff[:, :2],  axis=1)
error_rls = np.linalg.norm(ref_xy - states_rls[:, :2], axis=1)

plt.figure(figsize=(8, 4))
plt.plot(time, error_ff,  color='orange', label='Feedforward (no MPC)')
plt.plot(time, error_rls, color='green',  label='MPC + online RLS')
plt.xlabel('Time (s)')
plt.ylabel('Position error (m)')
plt.title('Position Tracking Error')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()

print(f'Feedforward      — mean error: {error_ff.mean():.4f} m, max: {error_ff.max():.4f} m')
print(f'MPC + online RLS — mean error: {error_rls.mean():.4f} m, max: {error_rls.max():.4f} m')

# --- Figure 3: Slip estimation ---
plt.figure(figsize=(8, 4))
plt.axhline(S_ACTUAL, color='black', linestyle='--', linewidth=1.5, label=f'True slip ({S_ACTUAL})')
plt.plot(time, slips_rls, color='green', label='RLS estimate')
plt.xlabel('Time (s)')
plt.ylabel('Slip')
plt.title('Online RLS Slip Estimation')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()

# --- Figure 4: Reference vs commanded wheel velocities ---
fig, axes = plt.subplots(1, 2, figsize=(12, 4))
axes[0].plot(time, vr_ref,            'r--',         label='Reference',        linewidth=1.5)
axes[0].plot(time, vr_ref + dvr_rls,  color='green', label='MPC + online RLS')
axes[0].set_xlabel('Time (s)')
axes[0].set_ylabel('Right Wheel Velocity (m/s)')
axes[0].set_title('Right Wheel Velocity: Reference vs Commanded')
axes[0].legend()
axes[0].grid(True)

axes[1].plot(time, vl_ref,            'r--',         label='Reference',        linewidth=1.5)
axes[1].plot(time, vl_ref + dvl_rls,  color='green', label='MPC + online RLS')
axes[1].set_xlabel('Time (s)')
axes[1].set_ylabel('Left Wheel Velocity (m/s)')
axes[1].set_title('Left Wheel Velocity: Reference vs Commanded')
axes[1].legend()
axes[1].grid(True)
plt.tight_layout()
plt.show()

