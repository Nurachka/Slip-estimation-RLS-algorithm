import sys
sys.path.append("..")
import os
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from mathematical_simulator_class.linear_mpc import LinearMPC
from mathematical_simulator_class.robot import Robot
from mathematical_simulator_class.file_reader import Analysis
from mathematical_simulator_class.feedforward import Feedforward
from mathematical_simulator_class.config import NOISE_STD_POSITION, NOISE_STD_ORIENTATION

file_reader = Analysis()
# 1.0,0.0,1.5786512774347865 these are the initial x,y,theta values because at time 0.0 sec, the robot is at (1,0) with theta close to pi/2
#otherwise the robot starts at origin (0,0) with 0 orientation which then makes lemniscate trajectory in vertical position not horizontal as targeted

current_dir = os.path.dirname(os.path.abspath(__file__))
trajectory_dir = os.path.join(current_dir, '..')
file_path = os.path.join(trajectory_dir, 'trajectories', 'lemniscate_trajectory.csv')

feedforward = Feedforward(file_reader.read_csv(file_path))

mpc_controller = LinearMPC(dt=0.05, wheel_base=0.5, N_horizon=10, s_l=0.04, s_r=0.04)


# --- Logging ---
log_actual     = []
log_reference  = []
log_error      = []
log_compensated = []
log_controls   = []

for timestep in range(len(feedforward.df)):
    vel_right, vel_left = feedforward.vel_at_timestep(timestep)
    
    # Get reference state from feedforward trajectory
    x, y = feedforward.x_y_at_timestep(timestep)
    theta = feedforward.theta_at_timestep(timestep)

    # Get actual state with noise and slip
    x_actual, y_actual, theta_actual = Robot(initial_x=1.0, initial_y=0.0, initial_theta=1.5786512774347865).forward_kinematics(vel_right, vel_left)
    x_noised = x_actual + np.random.normal(0, NOISE_STD_POSITION)
    y_noised = y_actual + np.random.normal(0, NOISE_STD_POSITION)
    theta_noised = theta_actual + np.random.normal(0, NOISE_STD_ORIENTATION)

    reference_state = np.array([x, y, theta])
    actual_state = np.array([x_noised, y_noised, theta_noised])
    error_state = mpc_controller.compute_error_state(actual_state, reference_state)

    A_list, B_list = [], []
    for i in range(mpc_controller.N):
        future_idx = min(timestep + i, len(feedforward.df) - 1)
        theta_ref_i = feedforward.theta_at_timestep(future_idx)
        vel_right_ref_i, vel_left_ref_i = feedforward.vel_at_timestep(future_idx)
        A_i, B_i = mpc_controller.define_AB_matrices(theta_ref_i, vel_right_ref_i, vel_left_ref_i)
        A_list.append(A_i)
        B_list.append(B_i)
    
    delta_vr, delta_vl = mpc_controller.solve(error_state, A_list, B_list)


    vel_right_comp = vel_right + delta_vr
    vel_left_comp = vel_left + delta_vl


    # Log data
    log_actual.append(actual_state.copy())
    log_reference.append(reference_state.copy())
    log_error.append(error_state.copy())
    log_controls.append((vel_right_comp, vel_left_comp))



#calculate the compensated trajectory
robot_comp = Robot(initial_x=1.0, initial_y=0.0, initial_theta=1.5786512774347865)
for i in range(len(log_controls)):
    vel_right_comp, vel_left_comp = log_controls[i]
    x_c, y_c, theta_c = robot_comp.ideal_forward_kinematics(vel_right_comp, vel_left_comp)
    compensated_state = np.array([x_c, y_c, theta_c])
    log_compensated.append(compensated_state)


log_actual = np.array(log_actual)
log_reference = np.array(log_reference)
log_compensated = np.array(log_compensated)
log_error = np.array(log_error)
log_controls = np.array(log_controls)

#plot the error in x and y position over time
plt.figure(figsize=(12, 6))
plt.subplot(2, 1, 1)
plt.plot(log_error[:, 0], label='X Error')
plt.xlabel('Time Step')
plt.ylabel('Error (m)')
plt.title('Error in X Position Over Time')
plt.legend()
plt.grid()
plt.subplot(2, 1, 2)
plt.plot(log_error[:, 1], label='Y Error')
plt.xlabel('Time Step')
plt.ylabel('Error (m)')
plt.title('Error in Y Position Over Time')
plt.legend()
plt.grid()
plt.tight_layout()
plt.show()

#plot the compensated velocities vs the original velocities
vel_right_comp_list = log_controls[:, 0]    
vel_left_comp_list = log_controls[:, 1]
vel_right_list = feedforward.df['right_vel'].values[:len(vel_right_comp_list)]
vel_left_list = feedforward.df['left_vel'].values[:len(vel_left_comp_list)]


plt.figure(figsize=(12, 6))
plt.subplot(2, 1, 1)   
plt.plot(vel_right_list, label='Original Right Wheel Velocity')
plt.plot(vel_right_comp_list, label='Compensated Right Wheel Velocity', linestyle='--')
plt.xlabel('Time Step')
plt.ylabel('Velocity (m/s)')
plt.title('Right Wheel Velocity Comparison')
plt.legend()
plt.grid()
plt.subplot(2, 1, 2)
plt.plot(vel_left_list, label='Original Left Wheel Velocity')
plt.plot(vel_left_comp_list, label='Compensated Left Wheel Velocity', linestyle='--')
plt.xlabel('Time Step')
plt.ylabel('Velocity (m/s)')
plt.title('Left Wheel Velocity Comparison')
plt.legend()
plt.grid()
plt.tight_layout()
plt.show()

x_target_list = feedforward.df['x'].tolist()
y_target_list = feedforward.df['y'].tolist()

#plot the trajectories
plt.figure(figsize=(8, 6))
plt.plot(x_target_list, y_target_list, label='Target trajectory', linestyle='-')
plt.plot(log_actual[:, 0], log_actual[:, 1], label='Actual Trajectory with Noise', linestyle='-.')
plt.plot(log_compensated[:, 0], log_compensated[:, 1], label='Compensated Trajectory', linestyle='--')
plt.xlabel('X Position (m)')
plt.ylabel('Y Position (m)')
plt.title('Trajectory Comparison')
plt.legend()
plt.axis('equal')
plt.grid()
plt.show()