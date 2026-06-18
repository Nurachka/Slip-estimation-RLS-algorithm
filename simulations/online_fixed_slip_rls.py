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
estimator = RecursiveLeastSquares(s0=np.array([0.0]), P0=10*np.eye(1), R=0.00436*np.eye(1))
slip = []
x_a, y_a, theta_a = 1.0, 0.0, 1.5786512774347865
comp_trajectory = []
vel_right_list = []
vel_left_list = []
vel_right_comp_list = []
vel_left_comp_list = []

theta_previous_noised = 1.5786512774347865
slip_previous = 0.0

for timestep in range(len(feedforward.df)):

    vel_right, vel_left = feedforward.vel_at_timestep(timestep)
    vel_right_list.append(vel_right)
    vel_left_list.append(vel_left)

    slip_clamped = max(min(slip_previous, 0.2), 0.0)
    vel_right_comp = vel_right / (1 - slip_clamped)
    vel_left_comp = vel_left / (1 - slip_clamped)
    vel_right_comp_list.append(vel_right_comp)
    vel_left_comp_list.append(vel_left_comp)

    x_a, y_a, theta_a = robot.forward_kinematics(vel_right_comp, vel_left_comp)
    x_noised, y_noised, theta_noised = robot.add_noise()

    estimator.predict_sim(theta_noised, theta_previous_noised, vel_right_comp, vel_left_comp, 0.05)
    theta_previous_noised = theta_noised

    comp_trajectory.append((x_a, y_a, theta_a))

    slip_previous = estimator.estimates[-1][0]
    slip.append(slip_previous)





x_target_list = feedforward.df['x'].tolist()
y_target_list = feedforward.df['y'].tolist()

plt.plot([x for x, y, theta in comp_trajectory], [y for x, y, theta in comp_trajectory], label='Compensated Robot Path', linestyle='--')
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
plt.plot([x for x, y, theta in comp_trajectory], [y for x, y, theta in comp_trajectory], label='Compensated trajectory', linestyle='--')
plt.xlabel('X Position (m)')
plt.ylabel('Y Position (m)')
plt.title('Reference vs Compensated Trajectory')
plt.legend()
plt.axis('equal')
plt.grid()
plt.show()

comp_x_aligned = [x_a] + [x for x, y, theta in comp_trajectory[:-1]]
comp_y_aligned = [y_a] + [y for x, y, theta in comp_trajectory[:-1]]
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

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
ax1.plot(vel_right_list, label='Nominal', linestyle='--')
ax1.plot(vel_right_comp_list, label='Compensated')
ax1.set_ylabel('Velocity (m/s)')
ax1.set_title('Right Wheel Velocity')
ax1.legend()
ax1.grid()
ax2.plot(vel_left_list, label='Nominal', linestyle='--')
ax2.plot(vel_left_comp_list, label='Compensated')
ax2.set_xlabel('Time Step')
ax2.set_ylabel('Velocity (m/s)')
ax2.set_title('Left Wheel Velocity')
ax2.legend()
ax2.grid()
plt.tight_layout()
plt.show()

#plot the angular velocity of the robot over time
angular_velocity = [(vel_right - vel_left) / 0.1 for vel_right, vel_left in zip(vel_right_comp_list, vel_left_comp_list)]
plt.plot(angular_velocity, label='Angular Velocity (compensated)')
plt.xlabel('Time Step')
plt.ylabel('Angular Velocity (rad/s)')
plt.title('Angular Velocity Over Time')     
plt.legend()
plt.grid()
plt.show()  