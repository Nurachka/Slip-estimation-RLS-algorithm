import sys
import os
sys.path.append("..")
from mathematical_simulator_class.robot import Robot
from mathematical_simulator_class.file_reader import Analysis
from mathematical_simulator_class.feedforward import Feedforward
from mathematical_simulator_class.recursive_least_square import RecursiveLeastSquares
from mathematical_simulator_class.compensator import Compensator
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

###Online implementation of the RLS algorithm with compensator and feedforward control###

file_reader = Analysis()
# 1.0,0.0,1.5786512774347865 these are the initial x,y,theta values because at time 0.0 sec, the robot is at (1,0) with theta close to pi/2
#otherwise the robot starts at origin (0,0) with 0 orientation which then makes lemniscate trajectory in vertical position not horizontal as targeted

robot = Robot(initial_x=1.0, initial_y=0.0, initial_theta=1.5786512774347865)
current_dir = os.path.dirname(os.path.abspath(__file__))
trajectory_dir = os.path.join(current_dir, '..', 'trajectories')
file_path = os.path.join(trajectory_dir, 'lemniscate_trajectory.csv')

feedforward = Feedforward(file_reader.read_csv(file_path))
estimator = RecursiveLeastSquares(s0=np.array([0.2]), P0=100*np.eye(1), R=1*np.eye(1,1))
slip = []
vel_right_comp_list = []
vel_left_comp_list = []

theta_previous_noised = 1.5786512774347865

for timestep in range(len(feedforward.df)):
    vel_right, vel_left = feedforward.vel_at_timestep(timestep)
    x,y,theta = robot.forward_kinematics(vel_right, vel_left)

    x_noised, y_noised, theta_noised = robot.add_noise()

    estimator.predict_sim(theta_noised, theta_previous_noised, vel_right, vel_left, 0.05)
    theta_previous_noised = theta_noised

    slip.append(estimator.estimates[timestep][0])
    slip_clamped = max(min(slip[timestep], 0.2), 0.0)
    vel_right_comp = vel_right / (1 - slip_clamped)
    vel_left_comp = vel_left / (1 - slip_clamped)
    vel_right_comp_list.append(vel_right_comp)
    vel_left_comp_list.append(vel_left_comp)

robot_comp = Robot(initial_x=1.0, initial_y=0.0, initial_theta=1.5786512774347865)
for timestep in range(len(vel_right_comp_list)):
    x_c, y_c, theta_c = robot_comp.forward_kinematics(vel_right_comp_list[timestep], vel_left_comp_list[timestep])

x_target_list = feedforward.df['x'].tolist()
y_target_list = feedforward.df['y'].tolist()

plt.plot(robot_comp.x_list, robot_comp.y_list, label='Compensated Robot Path', linestyle='--')
plt.plot(x_target_list, y_target_list, label='Target Path', linestyle=':')
plt.plot(robot.x_list, robot.y_list, label='Robot Path without compensation', linestyle=':')
plt.xlabel('X Position (m)')
plt.ylabel('Y Position (m)')
plt.title('Compensated Robot Path')
plt.legend()
plt.axis('equal')
plt.grid()
plt.show()

print("Estimated slip values:", slip[-1])

plt.plot(slip, label='Estimated Slip')
plt.xlabel('Time Step')
plt.ylabel('Slip')
plt.title('Estimated Slip Over Time')
plt.legend()
plt.grid()
plt.show()

#plot the target trajectory from calculated feedforward velocities
x_ref_list = x_target_list
y_ref_list = y_target_list
plt.plot(x_ref_list, y_ref_list, label='Reference trajectory', linestyle=':')
plt.plot(robot_comp.x_list, robot_comp.y_list, label='Compensated trajectory', linestyle='--')
plt.xlabel('X Position (m)')
plt.ylabel('Y Position (m)')
plt.title('Reference vs Compensated Trajectory')
plt.legend()
plt.axis('equal')
plt.grid()
plt.show()

comp_x_aligned = [robot_comp.initial_x] + robot_comp.x_list[:-1]
comp_y_aligned = [robot_comp.initial_y] + robot_comp.y_list[:-1]
error_comp = np.linalg.norm(np.column_stack((x_target_list, y_target_list)) - np.column_stack((comp_x_aligned, comp_y_aligned)), axis=1)
plt.figure(figsize=(10, 4))
plt.subplot(1, 2, 1)
plt.plot(error_comp, label='Tracking error (compensated vs target)')
plt.xlabel('Time Step')
plt.ylabel('Error (m)')
plt.title('Error Over Time')
plt.legend()
plt.grid()
plt.show()

print(f"Position error between compensated trajectory and target trajectory: min {error_comp.min():.3f} m, max {error_comp.max():.3f} m, mean {error_comp.mean():.3f} m")