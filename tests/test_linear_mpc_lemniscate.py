# Lemniscate trajectory tracking with LinearMPC under fixed slip and sensor noise.
#
# Simulates a robot following a lemniscate (figure-8) trajectory.
# Runs three scenarios, all with constant slip (s=0.1) and Gaussian position/orientation noise:
#   1. No MPC — feedforward only, slip uncompensated
#   2. MPC slip-unaware — controller does not know the true slip, noise applied to measurements
#   3. MPC slip-aware — controller is informed of true slip, noise applied to measurements
#
# Outputs: trajectory, position error, delta wheel velocities, actual vs reference velocities,
# and wheel accelerations (5 figures). Also prints final and mean position errors.

import sys
import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
sys.path.append("..")
from mathematical_simulator_class.linear_mpc import LinearMPC
from mathematical_simulator_class.config import NOISE_STD_POSITION, NOISE_STD_ORIENTATION

# --- Parameters ---
DT         = 0.05
WHEEL_BASE = 0.5
N          = 10
VR_MAX     = 0.8
VL_MAX     = 0.8
S_ACTUAL   = 0.1  # true slip in the plant (MPC blind by default)

# --- Load reference trajectory ---
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


def run_simulation(s_actual=0.0, s_mpc=0.0, seed=42, s_mpc_fn=None,
                   use_mpc=True, add_noise=False):
    np.random.seed(seed)

    mpc = LinearMPC(dt=DT, wheel_base=WHEEL_BASE, N_horizon=N,
                    vr_max=VR_MAX, vl_max=VL_MAX, s=s_mpc, du_max=None)

    x_a, y_a, theta_a = x_ref[0], y_ref[0], theta_ref[0]

    actual_states = []
    errors        = []
    delta_vr_list = []
    delta_vl_list = []

    for k in range(n_steps):
        actual_states.append((x_a, y_a, theta_a))

        if add_noise:
            x_meas     = x_a     + np.random.normal(0, NOISE_STD_POSITION)
            y_meas     = y_a     + np.random.normal(0, NOISE_STD_POSITION)
            theta_meas = theta_a + np.random.normal(0, NOISE_STD_ORIENTATION)
        else:
            x_meas, y_meas, theta_meas = x_a, y_a, theta_a

        error_state = mpc.compute_error_state(
            np.array([x_meas, y_meas, theta_meas]),
            np.array([x_ref[min(k + 1, n_steps - 1)], y_ref[min(k + 1, n_steps - 1)], theta_ref[min(k + 1, n_steps - 1)]])
        )
        errors.append(np.linalg.norm(
            mpc.compute_error_state(
                np.array([x_meas, y_meas, theta_meas]),
                np.array([x_ref[min(k + 1, n_steps - 1)], y_ref[min(k + 1, n_steps - 1)], theta_ref[min(k + 1, n_steps - 1)]])
            )[:2]
        ))

        if s_mpc_fn is not None:
            mpc.s = s_mpc_fn(k)

        if use_mpc:
            A_list, B_list = [], []
            vr_ref_list, vl_ref_list = [], []
            for i in range(N):
                future_idx = min(k + i, n_steps - 1)
                A_i, B_i   = mpc.define_AB_matrices(
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

        vr      = vr_ref[k] + delta_vr
        vl      = vl_ref[k] + delta_vl
        s       = s_actual 
        v_a     = (1 - s) * (vr + vl) / 2.0
        omega_a = (1 - s) * (vr - vl) / WHEEL_BASE
        x_a     += v_a * np.cos(theta_a) * DT
        y_a     += v_a * np.sin(theta_a) * DT
        theta_a += omega_a * DT

    return (np.array(actual_states),
            np.array(errors),
            delta_vr_list,
            delta_vl_list)


# --- Run scenarios ---
# states_clean, errors_clean, dvr_clean, dvl_clean = run_simulation()
# states_slip,  errors_slip,  dvr_slip,  dvl_slip  = run_simulation(s_actual=S_ACTUAL)
# states_comp,  errors_comp,  dvr_comp,  dvl_comp  = run_simulation(s_actual=S_ACTUAL,
#                                                                    s_mpc=S_ACTUAL)

# def slip_step_fn(k):
#     return 0.1 if k * DT < 10.0 else 0.2
# states_step,      errors_step,      dvr_step,      dvl_step      = run_simulation(s_actual_fn=slip_step_fn)
# states_step_comp, errors_step_comp, dvr_step_comp, dvl_step_comp = run_simulation(s_actual_fn=slip_step_fn,
#                                                                                    s_mpc_fn=slip_step_fn)

states_no_mpc,       errors_no_mpc,       dvr_no_mpc,       dvl_no_mpc       = run_simulation(s_actual=S_ACTUAL,
                                                                                              use_mpc=False)
states_mpc_unaware,  errors_mpc_unaware,  dvr_mpc_unaware,  dvl_mpc_unaware  = run_simulation(s_actual=S_ACTUAL,
                                                                                              add_noise=True)
states_mpc_noise,    errors_mpc_noise,    dvr_mpc_noise,    dvl_mpc_noise    = run_simulation(s_actual=S_ACTUAL,
                                                                                              s_mpc=S_ACTUAL,
                                                                                              add_noise=True)

labels = ['No MPC (constant slip)', 'MPC unaware of slip + noise', 'MPC aware of slip + noise']
colors = ['orange', 'blue', 'green']

# --- Figure 1: Trajectory comparison ---
plt.figure(figsize=(7, 7))
plt.plot(x_ref, y_ref, 'r--', label='Reference')
for states, label, color in zip([states_no_mpc, states_mpc_unaware, states_mpc_noise], labels, colors):
    plt.plot(states[:, 0], states[:, 1], color=color, label=label)
plt.xlabel('X (m)')
plt.ylabel('Y (m)')
plt.title('Lemniscate Trajectory Comparison')
plt.legend()
plt.axis('equal')
plt.grid(True)
plt.tight_layout()
plt.show()

# --- Figure 2: Position error over time ---
plt.figure(figsize=(7, 4))
for errors, label, color in zip([errors_no_mpc, errors_mpc_unaware, errors_mpc_noise], labels, colors):
    plt.plot(time, errors, color=color, label=label)
plt.xlabel('Time (s)')
plt.ylabel('Position Error (m)')
plt.title('Position Error Over Time')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()

# --- Figure 4: Reference vs Actual velocities ---
vr_actual_no_mpc      = vr_ref + np.array(dvr_no_mpc)
vl_actual_no_mpc      = vl_ref + np.array(dvl_no_mpc)
vr_actual_mpc_unaware = vr_ref + np.array(dvr_mpc_unaware)
vl_actual_mpc_unaware = vl_ref + np.array(dvl_mpc_unaware)
vr_actual_mpc_noise   = vr_ref + np.array(dvr_mpc_noise)
vl_actual_mpc_noise   = vl_ref + np.array(dvl_mpc_noise)

plt.figure(figsize=(12, 5))
plt.subplot(1, 2, 1)
plt.plot(time, vr_ref, 'r--', label='Reference', linewidth=1.5)
for vr_act, label, color in zip(
        [vr_actual_no_mpc, vr_actual_mpc_unaware, vr_actual_mpc_noise],
        labels, colors):
    plt.plot(time, vr_act, color=color, label=label, alpha=0.8)
plt.xlabel('Time (s)')
plt.ylabel('Right Wheel Velocity (m/s)')
plt.title('Right Wheel: Reference vs Actual')
plt.legend()
plt.grid(True)

plt.subplot(1, 2, 2)
plt.plot(time, vl_ref, 'r--', label='Reference', linewidth=1.5)
for vl_act, label, color in zip(
        [vl_actual_no_mpc, vl_actual_mpc_unaware, vl_actual_mpc_noise],
        labels, colors):
    plt.plot(time, vl_act, color=color, label=label, alpha=0.8)
plt.xlabel('Time (s)')
plt.ylabel('Left Wheel Velocity (m/s)')
plt.title('Left Wheel: Reference vs Actual')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()

# --- Figure 5: Wheel acceleration over time ---
def acceleration(v):
    return np.concatenate(([0.0], np.diff(v) / DT))

ar_no_mpc      = acceleration(vr_actual_no_mpc)
al_no_mpc      = acceleration(vl_actual_no_mpc)
ar_mpc_unaware = acceleration(vr_actual_mpc_unaware)
al_mpc_unaware = acceleration(vl_actual_mpc_unaware)
ar_mpc_noise   = acceleration(vr_actual_mpc_noise)
al_mpc_noise   = acceleration(vl_actual_mpc_noise)

plt.figure(figsize=(12, 5))
plt.subplot(1, 2, 1)
for ar, label, color in zip(
        [ar_no_mpc, ar_mpc_unaware, ar_mpc_noise],
        labels, colors):
    plt.plot(time, ar, color=color, label=label, alpha=0.8)
plt.xlabel('Time (s)')
plt.ylabel('Acceleration (m/s²)')
plt.title('Right Wheel Acceleration')
plt.legend()
plt.grid(True)

plt.subplot(1, 2, 2)
for al, label, color in zip(
        [al_no_mpc, al_mpc_unaware, al_mpc_noise],
        labels, colors):
    plt.plot(time, al, color=color, label=label, alpha=0.8)
plt.xlabel('Time (s)')
plt.ylabel('Acceleration (m/s²)')
plt.title('Left Wheel Acceleration')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()

# --- Summary ---
print(f"Final position error - No MPC (constant slip):   {errors_no_mpc[-1]:.6f} m")
print(f"Final position error - MPC unaware of slip+noise:     {errors_mpc_unaware[-1]:.6f} m")
print(f"Final position error - MPC aware of slip+noise:  {errors_mpc_noise[-1]:.6f} m")

print(f"Mean position error - No MPC (constant slip):   {np.mean(errors_no_mpc):.6f} m")
print(f"Mean position error - MPC unaware of slip+noise:      {np.mean(errors_mpc_unaware):.6f} m")
print(f"Mean position error - MPC aware of slip+noise:   {np.mean(errors_mpc_noise):.6f} m")



