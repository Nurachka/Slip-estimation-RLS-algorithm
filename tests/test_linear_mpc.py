import sys
import numpy as np
sys.path.append("..")
from mathematical_simulator_class.linear_mpc import LinearMPC

DT         = 0.05
WHEEL_BASE = 0.5
N          = 10
VR_MAX     = 0.5
VL_MAX     = 0.5
DELTA_U    = 0.05

mpc = LinearMPC(dt=DT, wheel_base=WHEEL_BASE, N_horizon=N,
                vr_max=VR_MAX, vl_max=VL_MAX, delta_u_max=DELTA_U)

def make_AB_lists(theta, vr, vl):
    A_k, B_k = mpc.define_AB_matrices(theta, vr, vl)
    return [A_k] * N, [B_k] * N


# --- Test 1: Initialization shapes ---
assert mpc.Q.shape   == (3, 3), "Q shape wrong"
assert mpc.R.shape   == (2, 2), "R shape wrong"
assert mpc.Q_N.shape == (3, 3), "Q_N shape wrong"
assert len(mpc.A) == N,         "Wrong number of A parameters"
assert len(mpc.B) == N,         "Wrong number of B parameters"
print("Test 1 passed: initialization shapes correct")


# --- Test 2: define_AB_matrices at theta=0 ---
# At theta=0, s=0: B_c[0,:]=0.5, B_c[1,:]=0, B_c[2,0]=1/(2l), B_c[2,1]=-1/(2l)
A_k, B_k = mpc.define_AB_matrices(theta=0.0, vel_right=0.2, vel_left=0.2)
assert A_k.shape == (3, 3), "A_k shape wrong"
assert B_k.shape == (3, 2), "B_k shape wrong"
assert np.isclose(B_k[0, 0],  DT / 2,                atol=1e-10), f"B_k[0,0] wrong: {B_k[0,0]}"
assert np.isclose(B_k[1, 0],  0.0,                   atol=1e-10), f"B_k[1,0] should be 0: {B_k[1,0]}"
assert np.isclose(B_k[2, 0],  DT / (2 * WHEEL_BASE), atol=1e-10), f"B_k[2,0] wrong: {B_k[2,0]}"
assert np.isclose(B_k[2, 1], -DT / (2 * WHEEL_BASE), atol=1e-10), f"B_k[2,1] wrong: {B_k[2,1]}"
print("Test 2 passed: A/B matrices at theta=0 correct")


# --- Test 3: define_AB_matrices at theta=pi/2 ---
# At theta=pi/2: B_c[0,:]=0, B_c[1,:]=0.5
A_k, B_k = mpc.define_AB_matrices(theta=np.pi / 2, vel_right=0.2, vel_left=0.2)
assert np.isclose(B_k[0, 0], 0.0,    atol=1e-10), f"B_k[0,0] at pi/2 should be 0, got {B_k[0,0]}"
assert np.isclose(B_k[1, 0], DT / 2, atol=1e-10), f"B_k[1,0] at pi/2 wrong: {B_k[1,0]}"
print("Test 3 passed: A/B matrices at theta=pi/2 correct")


# --- Test 4: compute_error_state basic ---
err = mpc.compute_error_state(
    actual_state    = np.array([1.5, 2.0, 0.5]),
    reference_state = np.array([1.0, 1.5, 0.2]),
)
assert np.isclose(err[0], 0.5, atol=1e-10), f"x error wrong: {err[0]}"
assert np.isclose(err[1], 0.5, atol=1e-10), f"y error wrong: {err[1]}"
assert np.isclose(err[2], 0.3, atol=1e-10), f"theta error wrong: {err[2]}"
print("Test 4 passed: compute_error_state correct")


# --- Test 5: compute_error_state angle wrapping ---
# (pi - 0.1) - (-pi + 0.1) = 2pi - 0.2, which must wrap to -0.2
err = mpc.compute_error_state(
    actual_state    = np.array([0.0, 0.0,  np.pi - 0.1]),
    reference_state = np.array([0.0, 0.0, -np.pi + 0.1]),
)
assert np.isclose(err[2], -0.2, atol=1e-6), f"Angle wrap failed: {err[2]:.6f}"
print("Test 5 passed: angle wrapping correct")


# --- Test 6: zero error → ~zero corrections ---
A_list, B_list = make_AB_lists(theta=0.0, vr=0.2, vl=0.2)
dvr, dvl = mpc.solve(np.zeros(3), A_list, B_list)
assert abs(dvr) < 1e-3, f"Expected ~0 correction for zero error, got dvr={dvr:.4f}"
assert abs(dvl) < 1e-3, f"Expected ~0 correction for zero error, got dvl={dvl:.4f}"
print(f"Test 6 passed: zero error → near-zero corrections (dvr={dvr:.4f}, dvl={dvl:.4f})")


# --- Test 7: output within velocity bounds ---
A_list, B_list = make_AB_lists(theta=0.0, vr=0.2, vl=0.2)
dvr, dvl = mpc.solve(np.array([0.3, 0.0, 0.0]), A_list, B_list)
assert abs(dvr) <= VR_MAX + 1e-6, f"dvr exceeds vr_max: {dvr:.4f}"
assert abs(dvl) <= VL_MAX + 1e-6, f"dvl exceeds vl_max: {dvl:.4f}"
print(f"Test 7 passed: output within velocity bounds (dvr={dvr:.4f}, dvl={dvl:.4f})")


# --- Test 8: acceleration constraint respected ---
u_prev = np.array([0.1, -0.1])
A_list, B_list = make_AB_lists(theta=0.0, vr=0.2, vl=0.2)
dvr, dvl = mpc.solve(np.array([0.3, 0.0, 0.0]), A_list, B_list, u_prev=u_prev)
assert abs(dvr - u_prev[0]) <= DELTA_U + 1e-3, f"dvr change violates delta_u_max: {abs(dvr - u_prev[0]):.4f}"
assert abs(dvl - u_prev[1]) <= DELTA_U + 1e-3, f"dvl change violates delta_u_max: {abs(dvl - u_prev[1]):.4f}"
print(f"Test 8 passed: acceleration constraint respected (Δvr={abs(dvr-u_prev[0]):.4f}, Δvl={abs(dvl-u_prev[1]):.4f})")


# --- Test 9: positive x error at theta=0 → decelerating correction ---
# B_k[0,:] = dt/2, so reducing positive x error requires dvr + dvl < 0
A_list, B_list = make_AB_lists(theta=0.0, vr=0.2, vl=0.2)
dvr, dvl = mpc.solve(np.array([0.3, 0.0, 0.0]), A_list, B_list, u_prev=np.zeros(2))
assert dvr + dvl < 0, f"Expected dvr+dvl < 0 for positive x error, got {dvr + dvl:.4f}"
print(f"Test 9 passed: positive x error → decelerating correction (dvr+dvl={dvr+dvl:.4f})")


# --- Test 10: closed-loop convergence over 50 steps ---
# Propagate the error through the linearized model with MPC corrections applied.
# Uses a relaxed delta_u_max so the rate constraint does not prevent convergence.
mpc_free = LinearMPC(dt=DT, wheel_base=WHEEL_BASE, N_horizon=N,
                     vr_max=VR_MAX, vl_max=VL_MAX, delta_u_max=0.5)
A_k, B_k = mpc_free.define_AB_matrices(theta=0.0, vel_right=0.2, vel_left=0.2)
A_list = [A_k] * N
B_list = [B_k] * N

error         = np.array([0.3, 0.0, 0.0])
u_prev        = np.zeros(2)
initial_norm  = np.linalg.norm(error[:2])

for _ in range(50):
    dvr, dvl = mpc_free.solve(error, A_list, B_list, u_prev=u_prev)
    u_prev   = np.array([dvr, dvl])
    error    = A_k @ error + B_k @ u_prev

final_norm = np.linalg.norm(error[:2])
assert final_norm < initial_norm, f"Error did not decrease: {initial_norm:.4f} → {final_norm:.4f} m"
print(f"Test 10 passed: position error reduced {initial_norm:.4f} → {final_norm:.4f} m over 50 steps")


print("\nAll LinearMPC tests passed.")
