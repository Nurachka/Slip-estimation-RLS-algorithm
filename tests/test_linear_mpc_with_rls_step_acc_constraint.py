# Lemniscate trajectory tracking with LinearMPC + online RLS slip estimation,
# under a hard wheel-acceleration constraint, with a STEP CHANGE in surface slip.
#
# Same setup as test_linear_mpc_with_rls_acc_constraint.py (robot moves first, noisy
# measurement, RLS estimates surface slip online from heading and feeds the unclipped
# estimate into the MPC each step, with a per-step slew-rate constraint du_max = ACC_MAX*DT
# and a heavier input-change cost S_DELTA capping wheel acceleration at ±ACC_MAX m/s²),
# with one difference: the true slip is not constant but a step function — it jumps from
# 0.0 to 0.2 at timestep 200, like in simulations/feedforward_rls_step_slip.py.
#
# Because a plain RLS adapts slowly to a sudden step (its covariance collapses), the RLS
# here uses a CONSTANT forgetting factor LAMBDA so it can re-converge after the step.
#
# Scenarios (true slip steps 0.0 -> 0.2 at step 200, Gaussian position/orientation noise):
#   1. Feedforward only (no MPC) — uncompensated baseline
#   2. MPC + online RLS, acceleration-limited    — slip-aware, du_max / S_DELTA
#   3. MPC slip-unaware (s=0), acceleration-limited — same constraint, no slip model
#
# Isolates whether the online slip estimate helps when the controller is rate-limited and
# the slip changes abruptly. Prints position/heading error stats, a slip-aware vs
# slip-unaware % improvement line, slip-estimation accuracy, and an acceleration check.
# (The unconstrained MPC+RLS scenario is commented out below.)

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
S_LOW        = 0.1                 # true slip before the step
S_HIGH       = 0.3                 # true slip after the step
SLIP_STEP    = 200                 # timestep at which slip jumps S_LOW -> S_HIGH
LAMBDA       = 0.95                # constant RLS forgetting factor (lets estimate track the step)
ACC_MAX      = 2.0                 # maximum wheel acceleration (m/s²)
DU_MAX       = ACC_MAX * DT        # equivalent per-step velocity change limit (m/s/step)
S_DELTA      = np.diag([50.0, 50.0])  # input-change cost weight matrix (penalizes ΔU each step)
STEADY_STATE_START    = 300        # slip-estimation steady-state window starts after the step
CONVERGENCE_THRESHOLD = 0.005      # convergence: |s_hat - s_true| < 0.005 (absolute slip units)
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


# --- True slip step profile ---
def slip_fn(k):
    return S_HIGH if k >= SLIP_STEP else S_LOW

true_slip = np.array([slip_fn(k) for k in range(n_steps)])


def run_simulation(slip_fn=None, use_mpc=True, use_rls=False, s_mpc=0.0,
                   use_noise=False, du_max=None, S_cost=None, seed=42):
    np.random.seed(seed)

    mpc = LinearMPC(dt=DT, wheel_base=WHEEL_BASE, N_horizon=N,
                    vr_max=VR_MAX, vl_max=VL_MAX, s=s_mpc, du_max=du_max, S=S_cost)

    rls        = RecursiveLeastSquares(s0=np.array([0.0]), P0=50.0 * np.eye(1), R=3.8e-5 * np.eye(1))
    theta_prev = theta_ref[0]

    x_a, y_a, theta_a = x_ref[0], y_ref[0], theta_ref[0]

    actual_states   = []
    vr_cmd_list     = []   # commanded velocities actually applied each step
    vl_cmd_list     = []
    slip_estimates  = []
    theta_meas_list = []

    for k in range(n_steps):

        # --- True slip at this timestep (step function) ---
        s_now = slip_fn(k) if slip_fn is not None else 0.0

        # --- Robot moves first (uses correction planned the previous step) ---
        if k == 0:
            vr = vr_ref[k]
            vl = vl_ref[k]
        else:
            vr = vr_ref[k] + delta_vr
            vl = vl_ref[k] + delta_vl
        vr_cmd_list.append(vr)
        vl_cmd_list.append(vl)

        v_a     = (1 - s_now) * (vr + vl) / 2.0
        omega_a = (1 - s_now) * (vr - vl) / WHEEL_BASE
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

        # --- RLS: uses vr/vl that just moved the robot (constant forgetting factor) ---
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

    return (np.array(actual_states),
            np.array(vr_cmd_list),
            np.array(vl_cmd_list),
            slip_estimates,
            np.array(theta_meas_list))


# --- Run scenarios ---
states_ff,  vrc_ff,  vlc_ff,  _,         _          = run_simulation(slip_fn=slip_fn, use_mpc=False, use_noise=True)
# states_unc, vrc_unc, vlc_unc, slips_unc, theta_unc  = run_simulation(slip_fn=slip_fn, use_mpc=True, use_rls=True,
#                                                                      use_noise=True, du_max=None, S_cost=None)
states_con, vrc_con, vlc_con, slips_con, theta_con  = run_simulation(slip_fn=slip_fn, use_mpc=True, use_rls=True,
                                                                     use_noise=True, du_max=DU_MAX, S_cost=S_DELTA)
# MPC slip-unaware (s=0), same acceleration constraint
states_con_su, vrc_con_su, vlc_con_su, _, _         = run_simulation(slip_fn=slip_fn, use_mpc=True, use_rls=False, s_mpc=0.0,
                                                                     use_noise=True, du_max=DU_MAX, S_cost=S_DELTA)

# --- Position tracking error ---
ref_xy    = np.column_stack((x_ref, y_ref))
error_ff     = np.linalg.norm(ref_xy - states_ff[:, :2],     axis=1)
# error_unc  = np.linalg.norm(ref_xy - states_unc[:, :2],    axis=1)
error_con    = np.linalg.norm(ref_xy - states_con[:, :2],    axis=1)
error_con_su = np.linalg.norm(ref_xy - states_con_su[:, :2], axis=1)

# --- Heading error (wrapped) ---
def wrap(a):
    return np.arctan2(np.sin(a), np.cos(a))
# heading_err_unc  = wrap(states_unc[:, 2]    - theta_ref)
heading_err_con    = wrap(states_con[:, 2]    - theta_ref)
heading_err_con_su = wrap(states_con_su[:, 2] - theta_ref)

# --- Acceleration from the commanded velocities actually applied ---
def acceleration(v):
    return np.concatenate(([0.0], np.diff(v) / DT))

# ar_unc, al_unc       = acceleration(vrc_unc),    acceleration(vlc_unc)
ar_con, al_con         = acceleration(vrc_con),    acceleration(vlc_con)
ar_con_su, al_con_su   = acceleration(vrc_con_su), acceleration(vlc_con_su)

# --- Figure 1: Trajectory ---
plt.figure(figsize=(7, 7))
plt.plot(x_ref, y_ref, 'r--', label='Reference', linewidth=1.5)
plt.plot(states_ff[:, 0],  states_ff[:, 1],  color='orange', label='Feedforward (no MPC)')
plt.plot(states_con_su[:, 0], states_con_su[:, 1], color='purple', label=f'MPC slip-unaware, acc-limited ({ACC_MAX} m/s²)')
plt.plot(states_con[:, 0], states_con[:, 1], color='green',  label=f'MPC+RLS, acc-limited ({ACC_MAX} m/s²)')
plt.xlabel('X (m)')
plt.ylabel('Y (m)')
plt.title('Lemniscate Trajectory — MPC + RLS with Acceleration Constraint (step slip)')
plt.legend()
plt.axis('equal')
plt.grid(True)
plt.tight_layout()
plt.show()

# --- Figure 2: Position tracking error ---
plt.figure(figsize=(8, 4))
#plt.plot(time, error_ff,  color='orange', label='Feedforward (no MPC)')
# plt.plot(time, error_unc, color='blue',   label='MPC+RLS, unconstrained', alpha=0.7)
plt.plot(time, error_con_su, color='purple', label='MPC slip-unaware, acc-limited', alpha=0.7)
plt.plot(time, error_con, color='green',  label='MPC+RLS, acc-limited')
plt.xlabel('Time (s)')
plt.ylabel('Position error (m)')
plt.title('Position Tracking Error')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()

# --- Figure 3: Heading tracking error ---
plt.figure(figsize=(8, 4))
plt.plot(time, np.degrees(heading_err_con_su), color='purple', label='MPC slip-unaware, acc-limited', alpha=0.7)
plt.plot(time, np.degrees(heading_err_con),    color='green',  label='MPC+RLS, acc-limited')
plt.xlabel('Time (s)')
plt.ylabel('Heading error (deg)')
plt.title('Heading Tracking Error')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()



# --- Figure 4: Slip estimation ---
plt.figure(figsize=(8, 4))
plt.plot(time, true_slip, 'k--', linewidth=1.5, label='True slip (step)')
plt.plot(time, slips_con, color='green', label='RLS estimate (acc-limited)')
plt.xlabel('Time (s)')
plt.ylabel('Slip')
plt.title('Online RLS Slip Estimation')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()

# --- Figure 5: Slip-unaware acceleration (acc-limited) ---
fig, axes = plt.subplots(1, 2, figsize=(12, 5))
axes[0].plot(time, ar_con_su, color='purple', label='Slip-unaware (acc-limited)')
axes[0].axhline( ACC_MAX, color='black', linestyle='--', linewidth=1.0, label=f'±{ACC_MAX} m/s² limit')
axes[0].axhline(-ACC_MAX, color='black', linestyle='--', linewidth=1.0)
axes[0].set_xlabel('Time (s)')
axes[0].set_ylabel('Acceleration (m/s²)')
axes[0].set_title('Right Wheel Acceleration')
axes[0].legend()
axes[0].grid(True)

axes[1].plot(time, al_con_su, color='purple', label='Slip-unaware (acc-limited)')
axes[1].axhline( ACC_MAX, color='black', linestyle='--', linewidth=1.0, label=f'±{ACC_MAX} m/s² limit')
axes[1].axhline(-ACC_MAX, color='black', linestyle='--', linewidth=1.0)
axes[1].set_xlabel('Time (s)')
axes[1].set_ylabel('Acceleration (m/s²)')
axes[1].set_title('Left Wheel Acceleration')
axes[1].legend()
axes[1].grid(True)
plt.suptitle('Wheel Acceleration — Slip-unaware (acc-limited)')
plt.tight_layout()
plt.show()

# --- Figure 6: MPC+RLS acceleration (acc-limited) ---
fig, axes = plt.subplots(1, 2, figsize=(12, 5))
axes[0].plot(time, ar_con, color='green', label='MPC+RLS (acc-limited)')
axes[0].axhline( ACC_MAX, color='black', linestyle='--', linewidth=1.0, label=f'±{ACC_MAX} m/s² limit')
axes[0].axhline(-ACC_MAX, color='black', linestyle='--', linewidth=1.0)
axes[0].set_xlabel('Time (s)')
axes[0].set_ylabel('Acceleration (m/s²)')
axes[0].set_title('Right Wheel Acceleration')
axes[0].legend()
axes[0].grid(True)

axes[1].plot(time, al_con, color='green', label='MPC+RLS (acc-limited)')
axes[1].axhline( ACC_MAX, color='black', linestyle='--', linewidth=1.0, label=f'±{ACC_MAX} m/s² limit')
axes[1].axhline(-ACC_MAX, color='black', linestyle='--', linewidth=1.0)
axes[1].set_xlabel('Time (s)')
axes[1].set_ylabel('Acceleration (m/s²)')
axes[1].set_title('Left Wheel Acceleration')
axes[1].legend()
axes[1].grid(True)
plt.suptitle('Wheel Acceleration — MPC+RLS (acc-limited)')
plt.tight_layout()
plt.show()

# --- Figure 7: Commanded wheel velocities (reference vs acc-limited) ---
fig, axes = plt.subplots(1, 2, figsize=(12, 4))
axes[0].plot(time, vr_ref,     'r--',          label='Reference', linewidth=1.5)
axes[0].plot(time, vrc_con_su, color='purple', label='Slip-unaware, acc-limited', alpha=0.7)
axes[0].plot(time, vrc_con,    color='green',  label='MPC+RLS, acc-limited')
axes[0].set_xlabel('Time (s)')
axes[0].set_ylabel('Right Wheel Velocity (m/s)')
axes[0].set_title('Right Wheel Velocity: Reference vs Commanded')
axes[0].legend()
axes[0].grid(True)

axes[1].plot(time, vl_ref,     'r--',          label='Reference', linewidth=1.5)
axes[1].plot(time, vlc_con_su, color='purple', label='Slip-unaware, acc-limited', alpha=0.7)
axes[1].plot(time, vlc_con,    color='green',  label='MPC+RLS, acc-limited')
axes[1].set_xlabel('Time (s)')
axes[1].set_ylabel('Left Wheel Velocity (m/s)')
axes[1].set_title('Left Wheel Velocity: Reference vs Commanded')
axes[1].legend()
axes[1].grid(True)
plt.tight_layout()
plt.show()

# --- Metrics ---
print(f'Feedforward                  — position mean error: {error_ff.mean():.4f} m, max: {error_ff.max():.4f} m')
# print(f'MPC+RLS unconstrained  — position mean error: {error_unc.mean():.4f} m, max: {error_unc.max():.4f} m')
print(f'MPC+RLS    acc-limited       — position mean error: {error_con.mean():.4f} m, max: {error_con.max():.4f} m')
print(f'MPC slip-unaware acc-limited — position mean error: {error_con_su.mean():.4f} m, max: {error_con_su.max():.4f} m')
print()
# print(f'MPC+RLS unconstrained  — heading mean |θ err|: {np.abs(heading_err_unc).mean():.4f} rad, max: {np.abs(heading_err_unc).max():.4f} rad')
print(f'MPC+RLS    acc-limited       — heading mean |θ err|: {np.abs(heading_err_con).mean():.4f} rad, max: {np.abs(heading_err_con).max():.4f} rad')
print(f'MPC slip-unaware acc-limited — heading mean |θ err|: {np.abs(heading_err_con_su).mean():.4f} rad, max: {np.abs(heading_err_con_su).max():.4f} rad')
print()
print(f'MPC+RLS acc-limited    — position RMSE: {np.sqrt(np.mean(error_con**2)):.4f} m, heading RMSE: {np.sqrt(np.mean(heading_err_con**2)):.4f} rad')
print(f'MPC+RLS acc-limited    — position std dev: {error_con.std():.4f} m, heading std dev: {np.degrees(heading_err_con.std()):.4f} deg')

# Improvement of slip-aware RLS over slip-unaware MPC (both acc-limited; positive = RLS better)
pos_impr  = (error_con_su.mean() - error_con.mean()) / error_con_su.mean() * 100
head_impr = (np.abs(heading_err_con_su).mean() - np.abs(heading_err_con).mean()) / np.abs(heading_err_con_su).mean() * 100
print(f'\nMPC+RLS vs slip-unaware (both acc-limited) — position mean improvement: {pos_impr:+.2f}%, heading mean improvement: {head_impr:+.2f}%')

# --- Slip estimation accuracy (acc-limited run; error against the time-varying true slip) ---
slips_arr = np.array(slips_con)
err_slip  = slips_arr - true_slip
rmse_full = np.sqrt(np.mean(err_slip ** 2))
mae_full  = np.mean(np.abs(err_slip))
err_ss    = err_slip[STEADY_STATE_START:]
rmse_ss   = np.sqrt(np.mean(err_ss ** 2))
mae_ss    = np.mean(np.abs(err_ss))
within = np.abs(err_slip) < CONVERGENCE_THRESHOLD
convergence_step = next((i for i in range(len(within)) if np.all(within[i:])), None)

print()
print(f'Slip estimation (full run)         — RMSE: {rmse_full:.4f}, MAE: {mae_full:.4f}')
print(f'Slip estimation (steps {STEADY_STATE_START}→end) — RMSE: {rmse_ss:.4f}, MAE: {mae_ss:.4f}')
if convergence_step is not None:
    print(f'Convergence time (±{CONVERGENCE_THRESHOLD})       — {time[convergence_step]:.2f} s (step {convergence_step})')
else:
    print(f'Convergence time (±{CONVERGENCE_THRESHOLD})       — did not converge')

# --- Acceleration comparison + constraint check ---
max_con    = max(np.abs(ar_con).max(),    np.abs(al_con).max())
max_con_su = max(np.abs(ar_con_su).max(), np.abs(al_con_su).max())
tol     = 1e-3  # OSQP solve tolerance
print()
print(f"Acceleration limit: ±{ACC_MAX} m/s²  (du_max = {DU_MAX} m/s/step)")
print(f"Max |accel| MPC+RLS      (acc-limited) — R: {np.abs(ar_con).max():.4f}, L: {np.abs(al_con).max():.4f}  (overall {max_con:.4f} m/s²)")
print(f"Max |accel| slip-unaware (acc-limited) — R: {np.abs(ar_con_su).max():.4f}, L: {np.abs(al_con_su).max():.4f}  (overall {max_con_su:.4f} m/s²)")
print(f"Constraint satisfied — MPC+RLS: {max_con <= ACC_MAX + tol}, slip-unaware: {max_con_su <= ACC_MAX + tol}")
