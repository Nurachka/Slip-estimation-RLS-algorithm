import sys
import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import time as _time
sys.path.append("..")
from mathematical_simulator_class.linear_mpc import LinearMPC

# --- Parameters ---
DT         = 0.05
WHEEL_BASE = 0.5
N          = 10
VR_MAX     = 0.5
VL_MAX     = 0.5
S_ACTUAL   = 0.1

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


def run_simulation(s_actual=0.0, s_mpc=0.0, use_mpc=True):
    mpc = LinearMPC(dt=DT, wheel_base=WHEEL_BASE, N_horizon=N,
                    vr_max=VR_MAX, vl_max=VL_MAX, s=s_mpc, du_max=None)

    x_a, y_a, theta_a = x_ref[0] + 0.1, y_ref[0] + 0.1, theta_ref[0]

    actual_states = []
    errors        = []
    delta_vr_list = []
    delta_vl_list = []

    for k in range(n_steps):
        start_time = _time.perf_counter()
        actual_states.append((x_a, y_a, theta_a))
        errors.append(np.linalg.norm(
            mpc.compute_error_state(
                np.array([x_a, y_a, theta_a]),
                np.array([x_ref[k], y_ref[k], theta_ref[k]])
            )[:2]
        ))

        if use_mpc:
            error_state = mpc.compute_error_state(
                np.array([x_a, y_a, theta_a]),
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
        #print(f"Time step {k+1}/{n_steps} - Computation time: {_time.perf_counter() - start_time:.4f} seconds")
        delta_vr_list.append(delta_vr)
        delta_vl_list.append(delta_vl)

        vr      = vr_ref[k] + delta_vr
        vl      = vl_ref[k] + delta_vl
        v_a     = (1 - s_actual) * (vr + vl) / 2.0
        omega_a = (1 - s_actual) * (vr - vl) / WHEEL_BASE
        x_a     += v_a * np.cos(theta_a) * DT
        y_a     += v_a * np.sin(theta_a) * DT
        theta_a += omega_a * DT

    return (np.array(actual_states),
            np.array(errors),
            np.array(delta_vr_list),
            np.array(delta_vl_list))


# --- Run scenarios ---
states_ff,       errors_ff,       dvr_ff,       dvl_ff       = run_simulation(s_actual=S_ACTUAL, use_mpc=False)
states_unaware,  errors_unaware,  dvr_unaware,  dvl_unaware  = run_simulation(s_actual=S_ACTUAL, s_mpc=0.0)
states_aware,    errors_aware,    dvr_aware,    dvl_aware    = run_simulation(s_actual=S_ACTUAL, s_mpc=S_ACTUAL)

labels = ['Feedforward (no MPC)', 'MPC slip-unaware', 'MPC slip-aware']
colors = ['orange', 'blue', 'green']

vr_total_ff      = vr_ref + dvr_ff
vl_total_ff      = vl_ref + dvl_ff
vr_total_unaware = vr_ref + dvr_unaware
vl_total_unaware = vl_ref + dvl_unaware
vr_total_aware   = vr_ref + dvr_aware
vl_total_aware   = vl_ref + dvl_aware

def acceleration(v):
    return np.concatenate(([0.0], np.diff(v) / DT))


# --- Figure 1: Trajectory ---
plt.figure(figsize=(7, 7))
plt.plot(x_ref, y_ref, 'r--', label='Reference', linewidth=1.5)
for states, label, color in zip([states_ff, states_unaware, states_aware], labels, colors):
    plt.plot(states[:, 0], states[:, 1], color=color, label=label)
plt.xlabel('X (m)')
plt.ylabel('Y (m)')
plt.title('Lemniscate Trajectory Comparison')
plt.legend()
plt.axis('equal')
plt.grid(True)
plt.tight_layout()
plt.show()

# --- Figure 2: Position error ---
plt.figure(figsize=(8, 4))
for errors, label, color in zip([errors_ff, errors_unaware, errors_aware], labels, colors):
    plt.plot(time, errors, color=color, label=label)
plt.xlabel('Time (s)')
plt.ylabel('Position Error (m)')
plt.title('Position Error Over Time')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()

# --- Figure 3: Control inputs (delta_vr, delta_vl) ---
fig, axes = plt.subplots(1, 2, figsize=(12, 4))
for dvr, label, color in zip([dvr_ff, dvr_unaware, dvr_aware], labels, colors):
    axes[0].plot(time, dvr, color=color, label=label)
axes[0].set_xlabel('Time (s)')
axes[0].set_ylabel('Delta VR (m/s)')
axes[0].set_title('Right Wheel Velocity Correction')
axes[0].legend()
axes[0].grid(True)

for dvl, label, color in zip([dvl_ff, dvl_unaware, dvl_aware], labels, colors):
    axes[1].plot(time, dvl, color=color, label=label)
axes[1].set_xlabel('Time (s)')
axes[1].set_ylabel('Delta VL (m/s)')
axes[1].set_title('Left Wheel Velocity Correction')
axes[1].legend()
axes[1].grid(True)
plt.tight_layout()
plt.show()

# --- Figure 4: Actual vs reference wheel velocities ---
fig, axes = plt.subplots(1, 2, figsize=(12, 4))
axes[0].plot(time, vr_ref, 'r--', label='Reference', linewidth=1.5)
for vr_tot, label, color in zip([vr_total_ff, vr_total_unaware, vr_total_aware], labels, colors):
    axes[0].plot(time, vr_tot, color=color, label=label, alpha=0.8)
axes[0].set_xlabel('Time (s)')
axes[0].set_ylabel('Right Wheel Velocity (m/s)')
axes[0].set_title('Right Wheel: Reference vs Actual')
axes[0].legend()
axes[0].grid(True)

axes[1].plot(time, vl_ref, 'r--', label='Reference', linewidth=1.5)
for vl_tot, label, color in zip([vl_total_ff, vl_total_unaware, vl_total_aware], labels, colors):
    axes[1].plot(time, vl_tot, color=color, label=label, alpha=0.8)
axes[1].set_xlabel('Time (s)')
axes[1].set_ylabel('Left Wheel Velocity (m/s)')
axes[1].set_title('Left Wheel: Reference vs Actual')
axes[1].legend()
axes[1].grid(True)
plt.tight_layout()
plt.show()

# --- Figure 5: Wheel acceleration ---
fig, axes = plt.subplots(1, 2, figsize=(12, 4))
for vr_tot, label, color in zip([vr_total_ff, vr_total_unaware, vr_total_aware], labels, colors):
    axes[0].plot(time, acceleration(vr_tot), color=color, label=label, alpha=0.8)
axes[0].set_xlabel('Time (s)')
axes[0].set_ylabel('Acceleration (m/s²)')
axes[0].set_title('Right Wheel Acceleration')
axes[0].legend()
axes[0].grid(True)

for vl_tot, label, color in zip([vl_total_ff, vl_total_unaware, vl_total_aware], labels, colors):
    axes[1].plot(time, acceleration(vl_tot), color=color, label=label, alpha=0.8)
axes[1].set_xlabel('Time (s)')
axes[1].set_ylabel('Acceleration (m/s²)')
axes[1].set_title('Left Wheel Acceleration')
axes[1].legend()
axes[1].grid(True)
plt.tight_layout()
plt.show()

# --- Summary ---
print(f"Final position error   - Feedforward:      {errors_ff[-1]:.6f} m")
print(f"Final position error   - MPC slip-unaware: {errors_unaware[-1]:.6f} m")
print(f"Final position error   - MPC slip-aware:   {errors_aware[-1]:.6f} m")
print()
print(f"Mean position error    - Feedforward:      {np.mean(errors_ff):.6f} m")
print(f"Mean position error    - MPC slip-unaware: {np.mean(errors_unaware):.6f} m")
print(f"Mean position error    - MPC slip-aware:   {np.mean(errors_aware):.6f} m")
print()
print(f"Max wheel acceleration - Feedforward:      {max(np.max(np.abs(acceleration(vr_total_ff))), np.max(np.abs(acceleration(vl_total_ff)))):.4f} m/s²")
print(f"Max wheel acceleration - MPC slip-unaware: {max(np.max(np.abs(acceleration(vr_total_unaware))), np.max(np.abs(acceleration(vl_total_unaware)))):.4f} m/s²")
print(f"Max wheel acceleration - MPC slip-aware:   {max(np.max(np.abs(acceleration(vr_total_aware))), np.max(np.abs(acceleration(vl_total_aware)))):.4f} m/s²")
