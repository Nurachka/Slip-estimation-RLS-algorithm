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
S_ACTUAL   = 0.1    # true slip in the plant (MPC always assumes s=0)

POS_NOISE_STD   = 0.0025    # m
THETA_NOISE_STD = 0.00436  # rad (~0.25 deg)

# --- Reference trajectory (straight line at 30 degrees) ---
ANGLE     = np.deg2rad(30)
time      = np.arange(N_STEPS) * DT
x_ref     = V_REF * np.cos(ANGLE) * time
y_ref     = V_REF * np.sin(ANGLE) * time
theta_ref = np.full(N_STEPS, ANGLE)
vr_ref    = np.full(N_STEPS, V_REF)
vl_ref    = np.full(N_STEPS, V_REF)


def run_simulation(s_actual=0.0, s_mpc=0.0, pos_std=0.0, theta_std=0.0, seed=42):
    np.random.seed(seed)

    mpc = LinearMPC(dt=DT, wheel_base=WHEEL_BASE, N_horizon=N,
                    vr_max=VR_MAX, vl_max=VL_MAX, s=s_mpc)

    x_a, y_a, theta_a = x_ref[0], y_ref[0] + 0.1, theta_ref[0]

    actual_states = []
    errors        = []
    delta_vr_list = []
    delta_vl_list = []

    for k in range(N_STEPS):
        actual_states.append((x_a, y_a, theta_a))

        error_state = mpc.compute_error_state(
            np.array([x_a, y_a, theta_a]),
            np.array([x_ref[k], y_ref[k], theta_ref[k]])
        )
        errors.append(np.linalg.norm(error_state[:2]))

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

        vr      = vr_ref[k] + delta_vr
        vl      = vl_ref[k] + delta_vl
        v_a     = (1 - s_actual) * (vr + vl) / 2.0
        omega_a = (1 - s_actual) * (vr - vl) / WHEEL_BASE
        x_a     += v_a * np.cos(theta_a) * DT + np.random.normal(0, pos_std)
        y_a     += v_a * np.sin(theta_a) * DT + np.random.normal(0, pos_std)
        theta_a += omega_a * DT               + np.random.normal(0, theta_std)

    return (np.array(actual_states),
            np.array(errors),
            delta_vr_list,
            delta_vl_list)


# --- Run four scenarios ---
states_clean, errors_clean, dvr_clean, dvl_clean = run_simulation()
states_slip,  errors_slip,  dvr_slip,  dvl_slip  = run_simulation(s_actual=S_ACTUAL)
states_noise, errors_noise, dvr_noise, dvl_noise  = run_simulation(s_actual=S_ACTUAL,
                                                                    pos_std=POS_NOISE_STD,
                                                                    theta_std=THETA_NOISE_STD)
states_comp,  errors_comp,  dvr_comp,  dvl_comp   = run_simulation(s_actual=S_ACTUAL, pos_std=POS_NOISE_STD, theta_std=THETA_NOISE_STD,
                                                                    s_mpc=S_ACTUAL)

labels = ['No Slip', 'Slip', 'Slip + Noise', 'Slip + Noise + MPC knows slip']
colors = ['blue', 'orange', 'green', 'red']

# --- Figure 1: Trajectory comparison ---
plt.figure(figsize=(7, 6))
plt.plot(x_ref, y_ref, 'r--', label='Reference')
for states, label, color in zip([states_clean, states_slip, states_noise, states_comp], labels, colors):
    plt.plot(states[:, 0], states[:, 1], color=color, label=label)
plt.xlabel('X (m)')
plt.ylabel('Y (m)')
plt.title('Trajectory Comparison')
plt.legend()
plt.axis('equal')
plt.tight_layout()
plt.show()

# --- Figure 2: Position error over time ---
plt.figure(figsize=(7, 4))
for errors, label, color in zip([errors_clean, errors_slip, errors_noise, errors_comp], labels, colors):
    plt.plot(time, errors, color=color, label=label)
plt.xlabel('Time (s)')
plt.ylabel('Position Error (m)')
plt.title('Position Error Over Time')
plt.legend()
plt.tight_layout()
plt.show()

# --- Figure 3: Delta VR and VL ---
plt.figure(figsize=(10, 4))
plt.subplot(1, 2, 1)
for dvr, label, color in zip([dvr_clean, dvr_slip, dvr_noise, dvr_comp], labels, colors):
    plt.plot(time, dvr, color=color, label=label)
plt.xlabel('Time (s)')
plt.ylabel('Delta VR (m/s)')
plt.title('Right Wheel Velocity Correction')
plt.legend()

plt.subplot(1, 2, 2)
for dvl, label, color in zip([dvl_clean, dvl_slip, dvl_noise, dvl_comp], labels, colors):
    plt.plot(time, dvl, color=color, label=label)
plt.xlabel('Time (s)')
plt.ylabel('Delta VL (m/s)')
plt.title('Left Wheel Velocity Correction')
plt.legend()
plt.tight_layout()
plt.show()

# --- Summary ---
print(f"Final position error — No Slip:            {errors_clean[-1]:.6f} m")
print(f"Final position error — Slip:               {errors_slip[-1]:.6f} m")
print(f"Final position error — Slip+Noise:         {errors_noise[-1]:.6f} m")
print(f"Final position error — Slip+Noise+MPC knows slip:{errors_comp[-1]:.6f} m")

# ---Mean errors over last 10 seconds---
print(f"Mean position error over last 10s — No Slip:            {np.mean(errors_clean[-int(10/DT):]):.6f} m")
print(f"Mean position error over last 10s — Slip:               {np.mean(errors_slip[-int(10/DT):]):.6f} m")
print(f"Mean position error over last 10s — Slip+Noise:         {np.mean(errors_noise[-int(10/DT):]):.6f} m")
print(f"Mean position error over last 10s — Slip+Noise+MPC knows slip:{np.mean(errors_comp[-int(10/DT):]):.6f} m")