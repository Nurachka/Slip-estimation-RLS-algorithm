import sys
import os
sys.path.append("..")
from mathematical_simulator.robot import Robot
from mathematical_simulator.file_reader import Analysis
from mathematical_simulator.feedforward import Feedforward
from mathematical_simulator.recursive_least_square import RecursiveLeastSquares
from mathematical_simulator.compensator import Compensator
import matplotlib.pyplot as plt
import numpy as np


file_reader = Analysis()
robot = Robot()
current_dir = os.path.dirname(os.path.abspath(__file__))
trajectory_dir = os.path.join(current_dir, 'trajectories' )
file_path = os.path.join(trajectory_dir, 'lemniscate_trajectory.csv')

print("File path:", file_path)
feedforward = Feedforward(file_reader.read_csv(file_path))
estimator = RecursiveLeastSquares(s0=np.array([1.1]), P0=100*np.eye(1), R=0.0004*np.eye(1))
slip = []
theta_previous_noised = 0.0
for timestep in range(len(feedforward.df)):
    vel_right, vel_left = feedforward.vel_at_timestep(timestep)
    print(f"Right wheel velocity: {vel_right}, Left wheel velocity: {vel_left}")
    x,y,theta = robot.forward_kinematics(vel_right, vel_left)
    print(f"Position after forward kinematics: x={x}, y={y}, theta={theta}")

    x_noised, y_noised, theta_noised = robot.add_noise()
    print(f"Noised position: x={x_noised}, y={y_noised}, theta={theta_noised}")
    
    # Calculate the slip using the recursive least squares estimator
    estimator.predict_sim(theta_noised, theta_previous_noised, vel_right, vel_left, 0.05)
    theta_previous_noised = theta_noised
    print(estimator.previousTimeStep)
    print(timestep)
    
    slip.append(estimator.estimates[timestep][0])






print("Estimated slip values:", slip[-1])

plt.plot(robot.x_list, robot.y_list, label='Original Path')
#plt.plot(robot.x_list_noised, robot.y_list_noised, label='Noised Path', linestyle='--')
plt.xlabel('X Position (m)')
plt.ylabel('Y Position (m)')
plt.title('Robot Path with and without Noise')
plt.legend()
plt.axis('equal')
plt.grid()
plt.show()


"""

#creating instance of the compensator class and modifying the velocities, thus x and y position of the robot
compensator = Compensator()
compensated_robot = Robot()
df = file_reader.read_csv(file_path)
print("Original DataFrame:", df)
#applying compensator for all the velocities in the dataframe
modified_df = compensator.modify_velocities(slip[-1], df)

compensated_feedforward = Feedforward(modified_df)
for timestep in range(len(compensated_feedforward.df)):
    vel_right, vel_left = compensated_feedforward.vel_at_timestep(timestep)
    x,y,theta = compensated_robot.forward_kinematics(vel_right, vel_left)

    x_noised, y_noised, theta_noised = compensated_robot.add_noise()
    #print(f"Compensated Noised position: x={x_noised}, y={y_noised}, theta={theta_noised}")

plt.plot(compensated_robot.x_list, compensated_robot.y_list, label='Compensated Path', linestyle='--')
plt.plot(compensated_robot.x_list_noised, compensated_robot.y_list_noised, label='Compensated Noised Path', linestyle=':')
#plt.plot(robot.x_list, robot.y_list, label='Original Path', color='blue')
plt.ylabel('Y Position (m)')
plt.title('Compensated Robot Path')
plt.legend()
plt.axis('equal')
plt.grid()
plt.show()


"""