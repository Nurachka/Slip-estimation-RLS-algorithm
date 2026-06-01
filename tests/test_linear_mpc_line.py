import sys
import numpy as np
import matplotlib.pyplot as plt
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
x_a, y_a, theta_a = x_ref[0], y_ref[0] + 0.05, theta_ref[0]

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
    v_a     = (vr + vl) / 2.0
    omega_a = (vr - vl) / WHEEL_BASE
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