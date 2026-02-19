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
trajectory_dir = os.path.join(current_dir, 'trajectories' )
file_path = os.path.join(trajectory_dir, 'lemniscate_trajectory.csv')

feedforward = Feedforward(file_reader.read_csv(file_path))
estimator = RecursiveLeastSquares(s0=np.array([0.0]), P0=100*np.eye(1), R=1*np.eye(1,1))
slip = []
vel_right_comp_list = []
vel_left_comp_list = []

theta_previous_noised = 1.5786512774347865

for timestep in range(len(feedforward.df)):
    vel_right, vel_left = feedforward.vel_at_timestep(timestep)
    #print(f"Right wheel velocity: {vel_right}, Left wheel velocity: {vel_left}")
    x,y,theta = robot.forward_kinematics(vel_right, vel_left)
    #print(f"Position after forward kinematics: x={x}, y={y}, theta={theta}")

    x_noised, y_noised, theta_noised = robot.add_noise()
    #print(f"Noised position: x={x_noised}, y={y_noised}, theta={theta_noised}")
    
    # Calculate the slip using the recursive least squares estimator
    estimator.predict_sim(theta_noised, theta_previous_noised, vel_right, vel_left, 0.05)
    theta_previous_noised = theta_noised
    
    slip.append(estimator.estimates[timestep][0])
    slip_clamped = max(min(slip[timestep], 0.2), 0.0)
    vel_right_comp = vel_right / (1 - slip_clamped)
    vel_left_comp = vel_left / (1 - slip_clamped)
    vel_right_comp_list.append(vel_right_comp)
    vel_left_comp_list.append(vel_left_comp)
#calculating the compensated trajectory
robot_comp = Robot(initial_x=1.0, initial_y=0.0, initial_theta=1.5786512774347865)
for timestep in range(len(vel_right_comp_list)):
    x_c, y_c, theta_c = robot_comp.forward_kinematics(vel_right_comp_list[timestep], vel_left_comp_list[timestep])


#extracting the x,y,theta values from the feedforward dataframe for comparison
for timestep in range(len(feedforward.df)):
    x_target, y_target= feedforward.x_y_at_timestep(timestep)
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

#plot the slip values over time
plt.plot(slip, label='Estimated Slip')
plt.xlabel('Time Step')
plt.ylabel('Slip')
plt.title('Estimated Slip Over Time')
plt.legend()
plt.grid()
plt.show()


