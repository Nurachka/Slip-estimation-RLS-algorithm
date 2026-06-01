import sys
import numpy as np
import matplotlib.pyplot as plt

np.random.seed(42)

POS_NOISE_STD   = 0.001    # m
THETA_NOISE_STD = 0.002  # rad (~0.25 deg)
sys.path.append("..")
from mathematical_simulator_class.linear_mpc import LinearMPC

# --- Parameters ---
DT         = 0.05
WHEEL_BASE = 0.5
N          = 10
VR_MAX     = 0.5
VL_MAX     = 0.5
V_REF      = 0.2     # constant forward speed (m/s)
N_STEPS    = 200     # 10 seconds of simulation
S_ACTUAL   = 0.01    # true slip in the plant (MPC assumes s=0)

# --- Step 1: Generate reference trajectory (straight line at 30 degrees) ---
ANGLE     = np.deg2rad(30)
time      = np.arange(N_STEPS) * DT
x_ref     = V_REF * np.cos(ANGLE) * time
y_ref     = V_REF * np.sin(ANGLE) * time
theta_ref = np.full(N_STEPS, ANGLE)
vr_ref    = np.full(N_STEPS, V_REF)
vl_ref    = np.full(N_STEPS, V_REF)



# # --- Step 2: Initialise MPC and actual robot ---
mpc = LinearMPC(dt=DT, wheel_base=WHEEL_BASE, N_horizon=N,
                vr_max=VR_MAX, vl_max=VL_MAX, s=0.0)

# 0.05 m offset in y, same heading as reference
x_a, y_a, theta_a = x_ref[0], y_ref[0] + 0.1, theta_ref[0]

actual_states      = []
errors             = []
delta_vr_list      = []
delta_vl_list      = []
delta_vr, delta_vl = 0.0, 0.0

# --- Step 3: Simulation loop ---
for k in range(N_STEPS):
    actual_states.append((x_a, y_a, theta_a))

    # Error state
    error_state = mpc.compute_error_state(
        np.array([x_a, y_a, theta_a]),
        np.array([x_ref[k], y_ref[k], theta_ref[k]])
    )
    errors.append(np.linalg.norm(error_state[:2]))

    # Build A_list, B_list over the prediction horizon
    A_list, B_list = [], []
    for i in range(N):
        future_idx = min(k + i, N_STEPS - 1)
        A_i, B_i   = mpc.define_AB_matrices(
            theta_ref[future_idx], vr_ref[future_idx], vl_ref[future_idx]
        )
        A_list.append(A_i)
        B_list.append(B_i)

    delta_vr, delta_vl = mpc.solve(error_state, A_list, B_list)
    delta_vr_list.append(delta_vr)
    delta_vl_list.append(delta_vl)

    # Integrate actual robot kinematics
    vr      = vr_ref[k] + delta_vr
    vl      = vl_ref[k] + delta_vl
    v_a     = (1 - S_ACTUAL) * (vr + vl) / 2.0
    omega_a = (1 - S_ACTUAL) * (vr - vl) / WHEEL_BASE
    x_a     += v_a * np.cos(theta_a) * DT 
    y_a     += v_a * np.sin(theta_a) * DT 
    theta_a += omega_a * DT            

actual_states = np.array(actual_states)
errors        = np.array(errors)

#plot trajectory comparison
plt.figure(figsize=(10, 5))
plt.subplot(1, 2, 1)
plt.plot(x_ref, y_ref, 'r--', label='Reference Trajectory')
plt.plot(actual_states[:, 0], actual_states[:, 1], 'b-', label='    Actual Trajectory')
plt.xlabel('X (m)')
plt.ylabel('Y (m)')
plt.title('Trajectory Comparison')
plt.legend()
plt.axis('equal')       

#plot error over time
plt.subplot(1, 2, 2)
plt.plot(time, errors, 'k-')
plt.xlabel('Time (s)')
plt.ylabel('Position Error (m)')
plt.title('Position Error Over Time')   
plt.tight_layout()
plt.show()

#plot delta_vr and delta_vl over time
plt.figure(figsize=(10, 4))
plt.subplot(1, 2, 1)
plt.plot(time, delta_vr_list, 'm-')
plt.xlabel('Time (s)')
plt.ylabel('Delta VR (m/s)')
plt.title('Right Wheel Velocity Correction')
plt.subplot(1, 2, 2)
plt.plot(time, delta_vl_list, 'c-')
plt.xlabel('Time (s)')
plt.ylabel('Delta VL (m/s)')
plt.title('Left Wheel Velocity Correction')
plt.tight_layout()
plt.show()

#plot actual vs reference velocities
plt.figure(figsize=(10, 4))
plt.subplot(1, 2, 1)
plt.plot(time, vr_ref, 'r--', label='VR Reference')
plt.plot(time, np.array(delta_vr_list) + vr_ref[:N_STEPS], 'b-', label='VR Actual')
plt.xlabel('Time (s)')
plt.ylabel('Right Wheel Velocity (m/s)')
plt.title('Right Wheel Velocity')
plt.legend()
plt.subplot(1, 2, 2)
plt.plot(time, vl_ref, 'r--', label='VL Reference')
plt.plot(time, np.array(delta_vl_list) + vl_ref[:N_STEPS], 'b-', label='VL Actual')
plt.xlabel('Time (s)')
plt.ylabel('Left Wheel Velocity (m/s)')
plt.title('Left Wheel Velocity')
plt.legend()
plt.tight_layout()
plt.show()

print(f"Final position error: {errors[-1]:.4f} m")
