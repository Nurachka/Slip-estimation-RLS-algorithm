import sys
import os
import time
sys.path.append("..")
from mathematical_simulator_class.robot import Robot
from mathematical_simulator_class.file_reader import Analysis
from mathematical_simulator_class.feedforward import Feedforward
from mathematical_simulator_class.recursive_least_square import RecursiveLeastSquares
from mathematical_simulator_class.compensator import Compensator
import matplotlib.pyplot as plt
import numpy as np

file_reader = Analysis()
# 1.0,0.0,1.5786512774347865 these are the initial x,y,theta values because at time 0.0 sec, the robot is at (1,0) with theta close to pi/2
#otherwise the robot starts at origin (0,0) with 0 orientation which then makes lemniscate trajectory in vertical position not horizontal as targeted

robot = Robot(initial_x=1.0, initial_y=0.0, initial_theta=1.5786512774347865)
current_dir = os.path.dirname(os.path.abspath(__file__))
trajectory_dir = os.path.join(current_dir, 'trajectories' )
file_path = os.path.join(trajectory_dir, 'lemniscate_trajectory.csv')

feedforward = Feedforward(file_reader.read_csv(file_path))
estimator = RecursiveLeastSquares(s0=np.array([0.0]), P0=10*np.eye(1), R=0.0004*np.eye(1,1))
slip = []

#plotting rls parameters over time
errors = []
estimationErrorCovarianceValues = [] 
kalmanGains = []

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
    #print(estimator.previousTimeStep)
    #print(timestep)
    
    slip.append(estimator.estimates[timestep][0])
    errors.append(estimator.errors[timestep][0])
    estimationErrorCovarianceValues.append(estimator.estimationErrorCovarianceMatrices[timestep][0])
    kalmanGains.append(estimator.gainMatrices[timestep][0])
   


#extracting the x,y,theta values from the feedforward dataframe for comparison
for timestep in range(len(feedforward.df)):
    x_target, y_target= feedforward.x_y_at_timestep(timestep)
    x_target_list = feedforward.df['x'].tolist()
    y_target_list = feedforward.df['y'].tolist()


#plot the x,y position of robot
plt.plot(x_target_list, y_target_list, label='Target trajectory', linestyle=':')
#plt.plot(robot.x_list_noised, robot.y_list_noised, label='Noised Robot Path', linestyle='--')
plt.xlabel('X Position (m)')
plt.ylabel('Y Position (m)')
plt.title('Target Robot trajectory')
plt.legend()
plt.axis('equal')
plt.grid()
plt.show()


print("Estimated slip values:", slip[-1])


plt.plot(robot.x_list, robot.y_list, label='Original trajectory with slip')
plt.plot(robot.x_list_noised, robot.y_list_noised, label='Noised trajectory', linestyle='--')
plt.xlabel('X Position (m)')
plt.ylabel('Y Position (m)')
plt.title('Robot trajectory with and without Noise')
plt.legend()
plt.axis('equal')
plt.grid()
plt.show()




#creating instance of the compensator class and modifying the velocities, thus x and y position of the robot
compensator = Compensator()
compensated_robot = Robot(initial_x=1.0, initial_y=0.0, initial_theta=1.5786512774347865)
df = file_reader.read_csv(file_path)
#applying compensator for all the velocities in the dataframe
modified_df = compensator.modify_velocities(slip[-1], df)

compensated_feedforward = Feedforward(modified_df)
for timestep in range(len(compensated_feedforward.df)):
    vel_right, vel_left = compensated_feedforward.vel_at_timestep(timestep)
    x,y,theta = compensated_robot.forward_kinematics(vel_right, vel_left)

plt.plot(compensated_robot.x_list, compensated_robot.y_list, label='Compensated trajectory', linestyle='--')
plt.plot(x_target_list, y_target_list, label='Target trajectory', linestyle=':')
plt.plot(robot.x_list, robot.y_list, label='Original trajectory with slip', linestyle='-.')
plt.ylabel('Y Position (m)')
plt.title('Compensated Robot trajectory')
plt.legend()
plt.axis('equal')
plt.grid()
plt.show()


#plot the slip values over time
plt.plot(slip, label='Estimated Slip')
plt.xlabel('Time Step')
plt.ylabel('Slip (m)')
plt.title('Estimated Slip Over Time')
plt.legend()
plt.grid()
plt.show()

#plot the errors over time
plt.plot(errors, label='Error Term')
plt.xlabel('Time Step')
plt.ylabel('Error Value (m)')
plt.title('Error Term Over Time')
plt.legend()
plt.grid()
plt.show()

#plot the estimation error covariance values over time
plt.plot(estimationErrorCovarianceValues, label='Estimation Error Covariance')
plt.xlabel('Time Step')
plt.ylabel('Covariance Value (m^2)')
plt.title('Estimation Error Covariance Over Time')
plt.legend()
plt.grid()
plt.show()

#plot the kalman gains over time
plt.plot(kalmanGains, label='Kalman Gain')
plt.xlabel('Time Step')
plt.ylabel('Kalman Gain Value')
plt.title('Kalman Gain Over Time')
plt.legend()
plt.grid()
plt.show()


#plot the theta difference over time
plt.plot(estimator.theta_diff, label='Theta Difference')
plt.xlabel('Time Step')
plt.ylabel('Theta Difference (radians)')
plt.title('Theta Difference Over Time')
plt.legend()
plt.grid()
plt.show()