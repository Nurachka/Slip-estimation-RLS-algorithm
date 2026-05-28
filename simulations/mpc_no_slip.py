import sys
import os
sys.path.append("..")
from mathematical_simulator_class.robot import Robot
from mathematical_simulator_class.file_reader import Analysis
from mathematical_simulator_class.feedforward import Feedforward
from mathematical_simulator_class.linear_mpc import LinearMPC
from mathematical_simulator_class.config import NOISE_STD_POSITION, NOISE_STD_ORIENTATION
import matplotlib.pyplot as plt
import numpy as np

# --- Constants ---
INITIAL_X     = 1.0
INITIAL_Y     = 0.0
INITIAL_THETA = 1.5786512774347865
TRUE_SLIP     = 0.2
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
# mpc_controller.s is fixed at 0.0 — slip is never fed into the prediction model
actual_robot   = Robot(initial_x=INITIAL_X, initial_y=INITIAL_Y, initial_theta=INITIAL_THETA)
mpc_controller = LinearMPC(dt=DELTA_T, wheel_base=0.5, N_horizon=10, s=0.0)

actual_states          = []
actual_states_no_noise = []
comp_velocities        = []
error_states           = []
delta_velocities       = []

# --- Single online loop ---
for k in range(n_steps):

    actual_robot.slip = TRUE_SLIP
    vel_right, vel_left = feedforward.vel_at_timestep(k)

    # Step actual robot — slip affects kinematics but MPC does not know about it
    if k == 0:
        x, y, theta = actual_robot.forward_kinematics(vel_right, vel_left)
    else:
        x, y, theta = actual_robot.forward_kinematics(vel_right+delta_vr, vel_left+delta_vl)
    
    x_n     = x     + np.random.normal(0, NOISE_STD_POSITION)
    y_n     = y     + np.random.normal(0, NOISE_STD_POSITION)
    theta_n = theta + np.random.normal(0, NOISE_STD_ORIENTATION)
    actual_states.append((x_n, y_n, theta_n))
    actual_states_no_noise.append((x, y, theta))

    # MPC with s=0.0 always — slip-unaware prediction model
    error_state = mpc_controller.compute_error_state(
        actual_states[k],
        reference_states[k]
    )

    A_list, B_list = [], []
    for i in range(mpc_controller.N):
        future_idx = min(k + i, n_steps - 1)
        theta_ref_i             = feedforward.theta_at_timestep(future_idx)
        vel_right_i, vel_left_i = feedforward.vel_at_timestep(future_idx)
        A_i, B_i = mpc_controller.define_AB_matrices(theta_ref_i, vel_right_i, vel_left_i)
        A_list.append(A_i)
        B_list.append(B_i)

    delta_vr, delta_vl = mpc_controller.solve(error_state, A_list, B_list)

    comp_velocities.append((vel_right + delta_vr, vel_left + delta_vl))
    error_states.append(error_state)
    delta_velocities.append((delta_vr, delta_vl))

actual_states   = np.array(actual_states)
actual_states_no_noise = np.array(actual_states_no_noise)
comp_velocities = np.array(comp_velocities)
error_states    = np.array(error_states)
delta_velocities = np.array(delta_velocities)




# --- Plot 1: Trajectory comparison ---
plt.figure(figsize=(8, 8))
plt.plot(reference_states[:, 0], reference_states[:, 1], linestyle=':',  label='Reference')
plt.plot(actual_states[:, 0],    actual_states[:, 1],    linestyle='--', label='Actual (no MPC)')
plt.plot(actual_states_no_noise[:, 0], actual_states_no_noise[:, 1], linestyle='-', label='Actual (no noise)')
plt.xlabel('X (m)')
plt.ylabel('Y (m)')
plt.title('Trajectory Comparison — MPC with no slip model')
plt.legend()
plt.gca().set_aspect('equal', adjustable='box')
plt.grid()
plt.tight_layout()
plt.show()



# --- Plot 3: Reference vs Compensated Velocities ---
vel_right_ref = np.array([feedforward.vel_at_timestep(k)[0] for k in range(n_steps)])
vel_left_ref  = np.array([feedforward.vel_at_timestep(k)[1] for k in range(n_steps)])
vel_right_comp = comp_velocities[:, 0]
vel_left_comp  = comp_velocities[:, 1]  
plt.figure(figsize=(10, 4))
plt.subplot(1, 2, 1)
plt.plot(vel_right_ref, label='Reference right wheel velocity')
plt.plot(vel_right_comp, label='Compensated right wheel velocity')
plt.xlabel('Timestep')
plt.ylabel('Velocity (m/s)')
plt.title('Right Wheel Velocity — Reference vs Compensated')
plt.legend()
plt.grid()
plt.subplot(1, 2, 2)
plt.plot(vel_left_ref, label='Reference left wheel velocity')
plt.plot(vel_left_comp, label='Compensated left wheel velocity')        
plt.xlabel('Timestep')
plt.ylabel('Velocity (m/s)')
plt.title('Left Wheel Velocity — Reference vs Compensated')
plt.legend()
plt.grid()
plt.tight_layout()
plt.show()

#plot the tracking error over time
error_comp   = np.linalg.norm(reference_states[:, :2] - actual_states[:, :2], axis=1)
plt.figure(figsize=(10, 4))
plt.plot(error_comp, label='Actual (no MPC)')
plt.xlabel('Timestep')
plt.ylabel('Position error (m)')
plt.title('Position Tracking Error — MPC with no slip model')
plt.legend()
plt.grid()
plt.tight_layout()
plt.show()

#print min, max, mean of the tracking error
print(f'Tracking error (no MPC) - Min: {error_comp.min():.4f} m, Max: {error_comp.max():.4f} m, Mean: {error_comp.mean():.4f} m')

#print the error state and delta velocities from 350-370 timesteps 
print("Error states and delta velocities from timesteps 350-370:")
for i in range(350, 371):
    print(f"Timestep {i}: Error state: {error_states[i]}, Delta velocities: {delta_velocities[i]}")