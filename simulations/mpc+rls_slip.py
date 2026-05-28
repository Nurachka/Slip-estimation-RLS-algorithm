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
WARMUP_STEPS  = 50    # s_hat forced to 0 until RLS has had time to converge
SLIP_CLIP     = 0.25
DELTA_T       = 0.05
LAMBDA        = 0.96

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
actual_robot   = Robot(initial_x=INITIAL_X, initial_y=INITIAL_Y, initial_theta=INITIAL_THETA)
rls            = RecursiveLeastSquares(s0=np.array([0.0]), P0=10.0 * np.eye(1), R=0.0004 * np.eye(1))
mpc_controller = LinearMPC(dt=DELTA_T, wheel_base=0.5, N_horizon=10, s=0.0)

theta_prev      = INITIAL_THETA
actual_states   = []
comp_velocities = []
error_states    = []
slip_estimates  = []

# --- Single online loop ---
# At each step k:
#   1. actual_robot steps with feedforward only  → gives the uncompensated trajectory
#   2. RLS updates from the new θ measurement    → produces s_hat for MPC
#   3. MPC uses s_hat in its linearised model    → computes velocity corrections
#   4. Corrected velocities are stored for comp_robot replay below
for k in range(n_steps):
    actual_robot.slip = TRUE_SLIP
    vel_right, vel_left = feedforward.vel_at_timestep(k)

    # Step actual robot with feedforward only (no MPC yet)
    if k == 0:
        x, y, theta = actual_robot.forward_kinematics(vel_right, vel_left)
    else: 
        x,y, theta = actual_robot.forward_kinematics(vel_right+delta_vr, vel_left+delta_vl)
    x_n     = x     + np.random.normal(0, NOISE_STD_POSITION)
    y_n     = y     + np.random.normal(0, NOISE_STD_POSITION)
    theta_n = theta + np.random.normal(0, NOISE_STD_ORIENTATION)
    actual_states.append((x_n, y_n, theta_n))

    # RLS update with the heading measurement just taken
    if k == 0:
        rls.predict_sim(
        theta_n, theta_prev, vel_right, vel_left, delta_t=DELTA_T
    )
    else: 
        rls.predict_sim_with_forgetting_factor(
        theta_n, theta_prev, vel_right+delta_vr, vel_left+delta_vl, delta_t=DELTA_T, lam=LAMBDA
    )
    theta_prev = theta_n

    # Slip estimate — hard-zeroed during warmup window
    s_hat = float(np.clip(rls.estimates[-1][0], -SLIP_CLIP, SLIP_CLIP))
    # if k < WARMUP_STEPS:
    #     s_hat = 0.0
    slip_estimates.append(s_hat)

    # Feed RLS estimate into MPC system model
    mpc_controller.s = s_hat

    error_state = mpc_controller.compute_error_state(
        actual_states[k],
        reference_states[k]
    )

    A_list, B_list = [], []
    for i in range(mpc_controller.N):
        future_idx = min(k + i, n_steps - 1)
        theta_ref_i           = feedforward.theta_at_timestep(future_idx)
        vel_right_i, vel_left_i = feedforward.vel_at_timestep(future_idx)
        A_i, B_i = mpc_controller.define_AB_matrices(theta_ref_i, vel_right_i, vel_left_i)
        A_list.append(A_i)
        B_list.append(B_i)

    delta_vr, delta_vl = mpc_controller.solve(error_state, A_list, B_list)

    comp_velocities.append((vel_right + delta_vr, vel_left + delta_vl))
    error_states.append(error_state)


actual_states   = np.array(actual_states)
comp_velocities = np.array(comp_velocities)
error_states    = np.array(error_states)
true_slip_list  = [TRUE_SLIP] * n_steps


# --- Plot 1: Trajectory comparison ---
plt.figure(figsize=(8, 8))
plt.plot(reference_states[:, 0], reference_states[:, 1], linestyle=':',  label='Reference')
plt.plot(actual_states[:, 0],    actual_states[:, 1],    linestyle='--', label='Actual (no MPC)')
plt.ylabel('Y (m)')
plt.title('Trajectory Comparison')
plt.legend()
plt.gca().set_aspect('equal', adjustable='box')
plt.grid()
plt.tight_layout()
plt.show()

# --- Plot 3: Slip estimation ---
plt.figure(figsize=(10, 4))
plt.plot(slip_estimates, label='RLS estimate')
plt.plot(true_slip_list, linestyle='--', label='True slip (step)')
plt.axvline(WARMUP_STEPS, color='orange', linestyle=':', linewidth=0.8, label=f'Warmup end (k={WARMUP_STEPS})')
plt.xlabel('Timestep')
plt.ylabel('Slip')
plt.title('Online RLS Slip Estimation fed into MPC')
plt.legend()
plt.grid()
plt.tight_layout()
plt.show()

# --- Plot 4: MPC velocity corrections ---
vel_right_ref = np.array([feedforward.vel_at_timestep(k)[0] for k in range(n_steps)])
vel_left_ref  = np.array([feedforward.vel_at_timestep(k)[1] for k in range(n_steps)])

# --- Plot 5: Compensated velocities vs reference velocities ---
plt.figure(figsize=(10, 4))
plt.subplot(2, 1, 1)
plt.plot(vel_right_ref, label='Reference right wheel velocity', linestyle='--')
plt.plot(comp_velocities[:, 0], label='Compensated right wheel velocity', alpha=0.7)
plt.xlabel('Timestep')
plt.ylabel('Velocity (m/s)')
plt.title('Right Wheel Velocity Comparison')
plt.legend()
plt.grid()
plt.subplot(2, 1, 2)
plt.plot(vel_left_ref, label='Reference left wheel velocity', linestyle='--')
plt.plot(comp_velocities[:, 1], label='Compensated left wheel velocity', alpha=0.7)
plt.xlabel('Timestep')
plt.ylabel('Velocity (m/s)')
plt.title('Left Wheel Velocity Comparison')
plt.legend()
plt.grid()
plt.tight_layout()
plt.show()

#plot tracking error over time
error_comp   = np.linalg.norm(reference_states[:, :2] - actual_states[:, :2], axis=1)
plt.figure(figsize=(10, 4))
plt.plot(error_comp, label='Position tracking error')
plt.xlabel('Timestep')
plt.ylabel('Position error (m)')
plt.title('Position Tracking Error over Time')
plt.legend()
plt.grid()
plt.tight_layout()
plt.show()

#print min, max, mean of the tracking error
print(f'Tracking error with MPC + RLS slip estimation - Min: {error_comp.min():.4f} m, Max: {error_comp.max():.4f} m, Mean: {error_comp.mean():.4f} m')