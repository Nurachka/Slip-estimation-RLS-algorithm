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

# exstract x,y,theta from feedforward trajectory for reference states
reference_states = []
for timestep in range(len(feedforward.df)):
    x = feedforward.x_y_at_timestep(timestep)[0]
    y = feedforward.x_y_at_timestep(timestep)[1]
    theta = feedforward.theta_at_timestep(timestep)
    reference_states.append((x, y, theta))
 
#modelling the same trajectory with noise and slip for actual states
robot = Robot(initial_x=1.0, initial_y=0.0, initial_theta=1.5786512774347865)
actual_states = []
for timestep in range(len(feedforward.df)):
    robot.slip = 0.2 
    vel_right, vel_left = feedforward.vel_at_timestep(timestep)
    x_actual, y_actual, theta_actual = robot.forward_kinematics(vel_right, vel_left)
    x_noised = x_actual + np.random.normal(0, NOISE_STD_POSITION)
    y_noised = y_actual + np.random.normal(0, NOISE_STD_POSITION)
    theta_noised = theta_actual + np.random.normal(0, NOISE_STD_ORIENTATION)
    actual_states.append((x_noised, y_noised, theta_noised))

mpc_controller = LinearMPC(dt=0.05, wheel_base=0.5, N_horizon=10, s=0.0)
error_states = []
comp_velocities = []

for timestep in range(len(feedforward.df)):
    #mpc_controller.s = 0.2
    reference_state = np.array(reference_states[timestep])
    actual_state = np.array(actual_states[timestep])
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

    vel_right_ref, vel_left_ref = feedforward.vel_at_timestep(timestep)
    
    vel_right_comp = vel_right_ref + delta_vr
    vel_left_comp = vel_left_ref + delta_vl

    comp_velocities.append((vel_right_comp, vel_left_comp))

    error_states.append(error_state)


# plotting the results
actual_states = np.array(actual_states)
reference_states = np.array(reference_states)
plt.figure(figsize=(12, 6))
plt.subplot(1, 2, 1)
plt.plot(reference_states[:, 0], reference_states[:, 1], label='Reference Trajectory', linestyle='--')
plt.plot(actual_states[:, 0], actual_states[:, 1], label='Actual Trajectory', alpha=0.7)
plt.title('Trajectory Comparison')
plt.xlabel('X Position')
plt.ylabel('Y Position')
plt.legend()
plt.grid()
plt.subplot(1, 2, 2)
plt.plot(error_states)
plt.title('Error State Over Time')
plt.xlabel('Time Step')
plt.ylabel('Error') 
plt.legend(['X Error', 'Y Error', 'Theta Error'])
plt.grid()
plt.tight_layout()
plt.show()

#plot ref velocities vs compensated velocities
comp_velocities = np.array(comp_velocities)
vel_right_ref = [feedforward.vel_at_timestep(t)[0] for t in range(len(feedforward.df))]
vel_left_ref = [feedforward.vel_at_timestep(t)[1] for t in range(len(feedforward.df))]
plt.figure(figsize=(12, 6))
plt.subplot(2, 1, 1)
plt.plot(vel_right_ref, label='Reference Right Wheel Velocity', linestyle='--')
plt.plot(comp_velocities[:, 0], label='Compensated Right Wheel Velocity', alpha=0.7)
plt.title('Right Wheel Velocity Comparison')
plt.xlabel('Time Step')
plt.ylabel('Velocity (m/s)')
plt.legend()
plt.grid()
plt.subplot(2, 1, 2)
plt.plot(vel_left_ref, label='Reference Left Wheel Velocity', linestyle='--')
plt.plot(comp_velocities[:, 1], label='Compensated Left Wheel Velocity', alpha=0.7)
plt.title('Left Wheel Velocity Comparison')
plt.xlabel('Time Step')
plt.ylabel('Velocity (m/s)')
plt.legend()
plt.grid()
plt.tight_layout()
plt.show()

#plot actual trajectory vs compensated trajectory
robot_comp = Robot(initial_x=1.0, initial_y=0.0, initial_theta=1.5786512774347865)
comp_trajectory = []
for vel_right_comp, vel_left_comp in comp_velocities:
    x_c, y_c, theta_c = robot_comp.ideal_forward_kinematics(vel_right_comp, vel_left_comp)
    comp_trajectory.append((x_c, y_c, theta_c))
comp_trajectory = np.array(comp_trajectory)

plt.figure(figsize=(12, 6))
plt.plot(reference_states[:, 0], reference_states[:, 1], label='Reference Trajectory', linestyle='--')
plt.plot(actual_states[:, 0], actual_states[:, 1], label='Actual Trajectory', alpha=0.7)
plt.plot(comp_trajectory[:, 0], comp_trajectory[:, 1], label='Compensated Trajectory', alpha=0.7)
plt.title('Trajectory Comparison')
plt.xlabel('X Position')
plt.ylabel('Y Position')
plt.legend()
plt.grid()
plt.tight_layout()
plt.show()

