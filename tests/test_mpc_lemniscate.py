import sys
import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
sys.path.append("..")
from mathematical_simulator_class.linear_mpc import LinearMPC

# --- Parameters ---
DT         = 0.05
WHEEL_BASE = 0.5
N          = 20
VR_MAX     = 0.5
VL_MAX     = 0.5
DELTA_U    = 0.05

# --- Step 1: Load trajectory ---
current_dir     = os.path.dirname(os.path.abspath(__file__))
trajectory_path = os.path.join(current_dir, '..', 'trajectories', 'lemniscate_trajectory.csv')
df = pd.read_csv(trajectory_path)

x_ref     = df['x'].values
y_ref     = df['y'].values
theta_ref = df['theta'].values
vr_ref    = df['right_vel'].values
vl_ref    = df['left_vel'].values
n_steps   = len(df)

# --- Step 2: Initialise MPC and actual robot ---
mpc = LinearMPC(dt=DT, wheel_base=WHEEL_BASE, N_horizon=N,
                vr_max=VR_MAX, vl_max=VL_MAX, delta_u_max=DELTA_U)

# 0.3 m offset in x and y from the reference start, same initial heading
x_a, y_a, theta_a = x_ref[0] + 0.01, y_ref[0] + 0.01, theta_ref[0]

actual_states      = []
errors             = []
delta_vr, delta_vl = 0.0, 0.0

# --- Step 3: Simulation loop ---
for k in range(n_steps):
    actual_states.append((x_a, y_a, theta_a))

    # Error state
    err_theta   = np.arctan2(np.sin(theta_a - theta_ref[k]), np.cos(theta_a - theta_ref[k]))
    error_state = np.array([x_a - x_ref[k], y_a - y_ref[k], err_theta])
    errors.append(np.linalg.norm(error_state[:2]))

    # Build A_list, B_list over the prediction horizon
    A_list, B_list = [], []
    for i in range(N):
        future_idx = min(k + i, n_steps - 1)
        A_i, B_i   = mpc.define_AB_matrices(
            theta_ref[future_idx], vr_ref[future_idx], vl_ref[future_idx]
        )
        A_list.append(A_i)
        B_list.append(B_i)      

    delta_vr, delta_vl = mpc.solve(error_state, A_list, B_list,
                                    u_prev=np.array([delta_vr, delta_vl]))

    # Move actual robot — ideal kinematics (no slip, no noise)
    vr      = vr_ref[k] + delta_vr
    vl      = vl_ref[k] + delta_vl
    v_a     = (vr + vl) / 2.0
    omega_a = (vr - vl) / WHEEL_BASE
    x_a     = x_a + v_a     * np.cos(theta_a) * DT
    y_a     = y_a + v_a     * np.sin(theta_a) * DT
    theta_a = theta_a + omega_a * DT

actual_states = np.array(actual_states)
errors        = np.array(errors)

# --- Step 4: Print and plot ---
print(f"Tracking error - min: {errors.min():.4f} m  "
      f"max: {errors.max():.4f} m  mean: {errors.mean():.4f} m")

# Plot 1: Trajectory comparison
plt.figure(figsize=(8, 8))
plt.plot(x_ref,                y_ref,                linestyle=':',  label='Reference')
plt.plot(actual_states[:, 0],  actual_states[:, 1],  linestyle='-',  label='Actual (MPC, no slip)')
plt.plot(x_ref[0],             y_ref[0],             'o', color='tab:blue',   markersize=8, label='Reference start')
plt.plot(actual_states[0, 0],  actual_states[0, 1],  's', color='tab:orange', markersize=8, label='Actual start')
plt.xlabel('X (m)')
plt.ylabel('Y (m)')
plt.title('Lemniscate Trajectory — MPC, no slip, no noise')
plt.legend()
plt.gca().set_aspect('equal', adjustable='box')
plt.grid()
plt.tight_layout()
plt.show()

# Plot 2: Tracking error over time
plt.figure(figsize=(10, 4))
plt.plot(errors, label='Position tracking error')
plt.xlabel('Timestep')
plt.ylabel('Error (m)')
plt.title('Position Tracking Error — MPC on Lemniscate')
plt.legend()
plt.grid()
plt.tight_layout()
plt.show()

# --- Assertions ---
assert errors[-1] < errors[0], \
    f"Final error not less than initial: {errors[0]:.4f} → {errors[-1]:.4f} m"
print(f"Assert 1 passed: final error {errors[-1]:.4f} m < initial error {errors[0]:.4f} m")

mean_first_100 = errors[:100].mean()
mean_last_100  = errors[-100:].mean()
assert mean_last_100 < mean_first_100, \
    f"Last-100 mean not less than first-100 mean: {mean_first_100:.4f} → {mean_last_100:.4f} m"
print(f"Assert 2 passed: last-100 mean {mean_last_100:.4f} m < first-100 mean {mean_first_100:.4f} m")


print("\nAll lemniscate MPC tests passed.")



#plot the actual vs reference velocity for right and left wheel
vel_right_ref = np.array(vr_ref)
vel_left_ref  = np.array(vl_ref)
vel_right_act = vel_right_ref + delta_vr
vel_left_act  = vel_left_ref  + delta_vl    
plt.figure(figsize=(10, 4))
plt.subplot(1, 2, 1)
plt.plot(vel_right_ref, label='Reference right wheel velocity')
plt.plot(vel_right_act, label='Actual right wheel velocity')
plt.xlabel('Timestep')
plt.ylabel('Velocity (m/s)')
plt.title('Right Wheel Velocity — Reference vs Actual')
plt.legend()
plt.grid()
plt.subplot(1, 2, 2)
plt.plot(vel_left_ref, label='Reference left wheel velocity')
plt.plot(vel_left_act, label='Actual left wheel velocity')
plt.xlabel('Timestep')
plt.ylabel('Velocity (m/s)')
plt.title('Left Wheel Velocity — Reference vs Actual')
plt.legend()
plt.grid()
plt.tight_layout()
plt.show()