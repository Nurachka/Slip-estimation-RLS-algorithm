import sys
import os
sys.path.append("..")
from mathematical_simulator_class.robot import Robot
from mathematical_simulator_class.file_reader import Analysis
from mathematical_simulator_class.feedforward import Feedforward
from mathematical_simulator_class.recursive_least_square import RecursiveLeastSquares
from mathematical_simulator_class.linear_mpc import LinearMPC
from mathematical_simulator_class.config import NOISE_STD_POSITION, NOISE_STD_ORIENTATION
import matplotlib.pyplot as plt
import numpy as np

# --- Constants ---
INITIAL_X     = 1.0
INITIAL_Y     = 0.0
INITIAL_THETA = 1.5786512774347865
TRUE_SLIP     = 0.2
WARMUP_STEPS  = 50
SLIP_CLIP     = 0.25
DELTA_T       = 0.05

# --- Load trajectory ---
file_reader    = Analysis()
current_dir    = os.path.dirname(os.path.abspath(__file__))
trajectory_dir = os.path.join(current_dir, '..', 'trajectories')
file_path      = os.path.join(trajectory_dir, 'lemniscate_trajectory.csv')
feedforward    = Feedforward(file_reader.read_csv(file_path))

n_steps = len(feedforward.df)

reference_states = []
for k in range(n_steps):
    x     = feedforward.x_y_at_timestep(k)[0]
    y     = feedforward.x_y_at_timestep(k)[1]
    theta = feedforward.theta_at_timestep(k)
    reference_states.append((x, y, theta))
reference_states = np.array(reference_states)

# --- Initialise components ---
# actual_robot: feedforward only — uncompensated baseline for comparison
# comp_robot:   receives MPC corrections every step — the closed-loop controlled robot
actual_robot   = Robot(initial_x=INITIAL_X, initial_y=INITIAL_Y, initial_theta=INITIAL_THETA)
comp_robot     = Robot(initial_x=INITIAL_X, initial_y=INITIAL_Y, initial_theta=INITIAL_THETA)
rls            = RecursiveLeastSquares(s0=np.array([0.0]), P0=10.0 * np.eye(1), R=0.0004 * np.eye(1))
mpc_controller = LinearMPC(dt=DELTA_T, wheel_base=0.5, N_horizon=10, s=0.0)

theta_prev         = INITIAL_THETA
actual_states      = []
comp_states        = []   # noisy observations — used by MPC and RLS
comp_true_states   = []   # clean robot positions — used for plotting and error
slip_estimates     = []
applied_velocities = []

# Both initialised to zero — no correction or slip estimate on the very first step
delta_vr, delta_vl = 0.0, 0.0
s_hat = 0.0

# --- Closed-loop online loop ---
# Fixes vs the original mpc+rls_slip.py:
#   FIX 1 — Closed loop: comp_robot receives the MPC correction at every step.
#            The MPC therefore observes the effect of its own corrections,
#            preventing unbounded error accumulation from the open-loop actual_robot.
#   FIX 2 — Consistent observations: RLS receives the velocities actually applied
#            to comp_robot (vel + delta), so its regressor matches the true motion.
#   FIX 3 — Consistent model: mpc_controller.s = s_hat AND comp_robot.slip = TRUE_SLIP,
#            so the B-matrix scaling (1-s) matches the robot's true response.
#            No ideal_forward_kinematics replay where corrections overshoot.
for k in range(n_steps):
    vel_right, vel_left = feedforward.vel_at_timestep(k)

    # Uncompensated robot — feedforward only, for comparison plot
    actual_robot.slip = TRUE_SLIP
    x_a, y_a, theta_a = actual_robot.forward_kinematics(vel_right, vel_left)
    actual_states.append((
        x_a     + np.random.normal(0, NOISE_STD_POSITION),
        y_a     + np.random.normal(0, NOISE_STD_POSITION),
        theta_a + np.random.normal(0, NOISE_STD_ORIENTATION),
    ))

    # Slip-compensated feedforward — divides by (1-s_hat) so the robot's effective
    # velocity matches the reference despite slip. MPC then corrects residual errors only.
    denom = max(1.0 - s_hat, 0.01)
    vel_ff_r = vel_right / denom
    vel_ff_l = vel_left  / denom

    # Compensated robot — apply correction computed at the previous step
    comp_robot.slip = TRUE_SLIP
    vr_applied = vel_ff_r + delta_vr
    vl_applied = vel_ff_l + delta_vl
    x_c, y_c, theta_c = comp_robot.forward_kinematics(vr_applied, vl_applied)
    x_cn     = x_c     + np.random.normal(0, NOISE_STD_POSITION)
    y_cn     = y_c     + np.random.normal(0, NOISE_STD_POSITION)
    theta_cn = theta_c + np.random.normal(0, NOISE_STD_ORIENTATION)
    comp_true_states.append((x_c,  y_c,  theta_c))
    comp_states.append((x_cn, y_cn, theta_cn))
    applied_velocities.append((vr_applied, vl_applied))

    # RLS update — uses comp_robot heading and the velocities actually applied (FIX 2)
    rls.predict_sim(theta_cn, theta_prev, vr_applied, vl_applied, delta_t=DELTA_T)
    theta_prev = theta_cn

    s_hat = float(np.clip(rls.estimates[-1][0], -SLIP_CLIP, SLIP_CLIP))
    if k < WARMUP_STEPS:
        s_hat = 0.0
    slip_estimates.append(s_hat)

    # MPC error from comp_robot's state — not from uncompensated actual_robot (FIX 1)
    mpc_controller.s = s_hat          # model matches robot's slip (FIX 3)
    error_state = mpc_controller.compute_error_state(
        np.array([x_cn, y_cn, theta_cn]),
        reference_states[k]
    )

    A_list, B_list = [], []
    for i in range(mpc_controller.N):
        future_idx              = min(k + i, n_steps - 1)
        theta_ref_i             = feedforward.theta_at_timestep(future_idx)
        vel_right_i, vel_left_i = feedforward.vel_at_timestep(future_idx)
        A_i, B_i = mpc_controller.define_AB_matrices(theta_ref_i, vel_right_i, vel_left_i)
        A_list.append(A_i)
        B_list.append(B_i)

    delta_vr, delta_vl = mpc_controller.solve(error_state, A_list, B_list)

actual_states      = np.array(actual_states)
comp_states        = np.array(comp_states)
comp_true_states   = np.array(comp_true_states)
applied_velocities = np.array(applied_velocities)
true_slip_list     = [TRUE_SLIP] * n_steps

# --- Plot 1: Trajectory comparison ---
plt.figure(figsize=(8, 8))
plt.plot(reference_states[:, 0], reference_states[:, 1], linestyle=':',  label='Reference')
plt.plot(actual_states[:, 0],    actual_states[:, 1],    linestyle='--', label='Actual (no MPC)', color='gray')
plt.plot(comp_true_states[:, 0], comp_true_states[:, 1], linestyle='-.', label='MPC + RLS (fixed)', color='tab:blue')
plt.xlabel('X (m)')
plt.ylabel('Y (m)')
plt.title('Trajectory Comparison — MPC + RLS (closed-loop)')
plt.legend()
plt.gca().set_aspect('equal', adjustable='box')
plt.grid()
plt.tight_layout()
plt.show()

# --- Plot 2: Slip estimation ---
plt.figure(figsize=(10, 4))
plt.plot(slip_estimates, label='RLS estimate', color='tab:blue')
plt.plot(true_slip_list, linestyle='--', label='True slip', color='tab:red')
plt.axvline(WARMUP_STEPS, color='gray', linestyle=':', linewidth=0.8, label=f'Warmup end (k={WARMUP_STEPS})')
plt.xlabel('Timestep')
plt.ylabel('Slip')
plt.title('RLS Slip Estimation (closed-loop observations)')
plt.legend()
plt.grid()
plt.tight_layout()
plt.show()

# --- Plot 3: Applied velocities vs reference ---
vel_right_ref = np.array([feedforward.vel_at_timestep(k)[0] for k in range(n_steps)])
vel_left_ref  = np.array([feedforward.vel_at_timestep(k)[1] for k in range(n_steps)])

plt.figure(figsize=(10, 4))
plt.subplot(2, 1, 1)
plt.plot(vel_right_ref,            label='Reference right', linestyle='--')
plt.plot(applied_velocities[:, 0], label='Applied right (FF-compensated + MPC)', alpha=0.7)
plt.xlabel('Timestep')
plt.ylabel('Velocity (m/s)')
plt.title('Right Wheel Velocity')
plt.legend()
plt.grid()
plt.subplot(2, 1, 2)
plt.plot(vel_left_ref,             label='Reference left', linestyle='--')
plt.plot(applied_velocities[:, 1], label='Applied left (FF-compensated + MPC)',  alpha=0.7)
plt.xlabel('Timestep')
plt.ylabel('Velocity (m/s)')
plt.title('Left Wheel Velocity')
plt.legend()
plt.grid()
plt.tight_layout()
plt.show()

# --- Plot 4: Position tracking error ---
comp_true_aligned = np.vstack([[INITIAL_X, INITIAL_Y, INITIAL_THETA], comp_true_states[:-1]])
error_comp = np.linalg.norm(reference_states[:, :2] - comp_true_aligned[:, :2], axis=1)

plt.figure(figsize=(10, 4))
plt.plot(error_comp, label='MPC + RLS (fixed)', color='tab:blue')
plt.axvline(WARMUP_STEPS, color='gray', linestyle=':', linewidth=0.8, label=f'Warmup end (k={WARMUP_STEPS})')
plt.xlabel('Timestep')
plt.ylabel('Position error (m)')
plt.title('Position Tracking Error — MPC + RLS (closed-loop)')
plt.legend()
plt.grid()
plt.tight_layout()
plt.show()

print(f"Position error (MPC + RLS fixed): min {error_comp.min():.3f} m, max {error_comp.max():.3f} m, mean {error_comp.mean():.3f} m")
