import sys
import numpy as np
sys.path.append("..")
from mathematical_simulator_class.linear_mpc import LinearMPC

# --- Parameters ---
DT         = 0.05
WHEEL_BASE = 0.5
N          = 10
VR_MAX     = 0.5
VL_MAX     = 0.5
DELTA_U    = 0.05
V_REF      = 0.2   # vel_right = vel_left → straight line, theta stays 0
N_STEPS    = 100

mpc = LinearMPC(dt=DT, wheel_base=WHEEL_BASE, N_horizon=N,
                vr_max=VR_MAX, vl_max=VL_MAX, delta_u_max=DELTA_U)


# --- Step 1: Reference trajectory ---
# Straight line along x: propagate ideal kinematics from (0, 0, 0)
# with vel_right = vel_left = V_REF (no rotation)
reference = [(0.0, 0.0, 0.0)]
for _ in range(N_STEPS - 1):
    x_r, y_r, theta_r = reference[-1]
    v     = (V_REF + V_REF) / 2.0
    omega = (V_REF - V_REF) / WHEEL_BASE  # 0 — straight line
    reference.append((
        x_r     + v     * np.cos(theta_r) * DT,
        y_r     + v     * np.sin(theta_r) * DT,
        theta_r + omega * DT,
    ))


# --- Step 2: Actual robot — 0.2 m offset in x from reference start ---
x_a, y_a, theta_a = 0.2, 0.0, 0.0


# --- Step 3: Simulation loop ---
delta_vr, delta_vl = 0.0, 0.0
errors = []

for k in range(N_STEPS):
    x_r, y_r, theta_r = reference[k]

    # Error state (actual − reference), heading wrapped to (−π, π]
    err_theta   = np.arctan2(np.sin(theta_a - theta_r), np.cos(theta_a - theta_r))
    error_state = np.array([x_a - x_r, y_a - y_r, err_theta])
    errors.append(np.linalg.norm(error_state[:2]))

    # A, B matrices linearised around the current reference point
    A_k, B_k = mpc.define_AB_matrices(theta_r, V_REF, V_REF)
    A_list    = [A_k] * N
    B_list    = [B_k] * N

    delta_vr, delta_vl = mpc.solve(error_state, A_list, B_list,
                                    u_prev=np.array([delta_vr, delta_vl]))

    # Move actual robot — ideal kinematics (no slip, no noise)
    vr      = V_REF + delta_vr
    vl      = V_REF + delta_vl
    v_a     = (vr + vl) / 2.0
    omega_a = (vr - vl) / WHEEL_BASE
    x_a     = x_a + v_a     * np.cos(theta_a) * DT
    y_a     = y_a + v_a     * np.sin(theta_a) * DT
    theta_a = theta_a + omega_a * DT

    if k % 25 == 0:
        print(f"  step {k:3d}: position error = {errors[-1]:.4f} m  "
              f"(delta_vr={delta_vr:+.4f}, delta_vl={delta_vl:+.4f})")


# --- Step 4: Assertions ---
print(f"\nInitial position error : {errors[0]:.4f} m")
print(f"Final   position error : {errors[-1]:.4f} m")

assert errors[-1] < errors[0], \
    f"MPC did not reduce error: {errors[0]:.4f} → {errors[-1]:.4f} m"

print(f"\nTest passed: MPC reduced position error from "
      f"{errors[0]:.4f} m to {errors[-1]:.4f} m over {N_STEPS} steps.")

