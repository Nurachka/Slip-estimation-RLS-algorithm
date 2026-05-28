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

file_reader = Analysis()
# 1.0,0.0,1.5786512774347865 these are the initial x,y,theta values because at time 0.0 sec, the robot is at (1,0) with theta close to pi/2
#otherwise the robot starts at origin (0,0) with 0 orientation which then makes lemniscate trajectory in vertical position not horizontal as targeted

robot = Robot(initial_x=1.0, initial_y=0.0, initial_theta=1.5786512774347865)
current_dir = os.path.dirname(os.path.abspath(__file__))
trajectory_dir = os.path.join(current_dir, '..', 'trajectories')
file_path = os.path.join(trajectory_dir, 'lemniscate_trajectory.csv')

feedforward = Feedforward(file_reader.read_csv(file_path))
estimator = RecursiveLeastSquares(s0=np.array([0.0]), P0=10*np.eye(1,1), R=0.00436*np.eye(1,1))
slip = []

errors = []
estimationErrorCovarianceValues = []
kalmanGains = []
reference_state_list = []

theta_previous_noised = 1.5786512774347865
for timestep in range(len(feedforward.df)):
    vel_right, vel_left = feedforward.vel_at_timestep(timestep)
    x,y,theta = robot.forward_kinematics(vel_right, vel_left)

    x_noised, y_noised, theta_noised = robot.add_noise()

    estimator.predict_sim(theta_noised, theta_previous_noised, vel_right, vel_left, 0.05)
    theta_previous_noised = theta_noised
    
    reference_state_list.append(np.array([x, y, theta]))
    slip.append(estimator.estimates[timestep][0])
    errors.append(estimator.errors[timestep][0])
    estimationErrorCovarianceValues.append(estimator.estimationErrorCovarianceMatrices[timestep][0])
    kalmanGains.append(estimator.gainMatrices[timestep][0])

x_target_list = feedforward.df['x'].tolist()
y_target_list = feedforward.df['y'].tolist()

plt.plot(x_target_list, y_target_list, label='Target trajectory', linestyle=':')
plt.xlabel('X Position (m)')
plt.ylabel('Y Position (m)')
plt.title('Target Robot trajectory')
plt.legend()
plt.axis('equal')
plt.grid()
plt.show()

# plot the target trajectory from calculated feedforward velocities
x_ref_list = []
y_ref_list = []
for state in reference_state_list:
    x_ref_list.append(state[0])
    y_ref_list.append(state[1])



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

compensator = Compensator()
compensated_robot = Robot(initial_x=1.0, initial_y=0.0, initial_theta=1.5786512774347865)
df = file_reader.read_csv(file_path)
modified_df = compensator.modify_velocities(slip[-1], df)

compensated_feedforward = Feedforward(modified_df)
for timestep in range(len(compensated_feedforward.df)):
    vel_right, vel_left = compensated_feedforward.vel_at_timestep(timestep)
    x,y,theta = compensated_robot.forward_kinematics(vel_right, vel_left)
    

plt.plot(compensated_robot.x_list, compensated_robot.y_list, label='Compensated trajectory', linestyle='--')
plt.plot(x_target_list, y_target_list, label='Target trajectory', linestyle=':')
plt.plot(robot.x_list, robot.y_list, label='Original trajectory with slip', linestyle='-.')
plt.xlabel('X Position (m)')
plt.ylabel('Y Position (m)')
plt.title('Compensated Robot trajectory')
plt.legend()
plt.axis('equal')
plt.grid()
plt.show()

#plot compensated theta vs reference theta
plt.plot(compensated_robot.theta_list, label='Compensated trajectory', linestyle='-.')
plt.xlabel('Time Step')
plt.ylabel('Theta (radians)')
plt.title('Compensated Theta vs Reference Theta')
plt.legend()
plt.grid()
plt.show()

angular_velocity = [(compensated_robot.theta_list[i] - compensated_robot.theta_list[i-1]) / 0.05
                    for i in range(1, len(compensated_robot.theta_list))]
angular_velocity_ref = [(robot.theta_list[i] - robot.theta_list[i-1]) / 0.05
                        for i in range(1, len(robot.theta_list))]

#plot error between compensated trajectory and target trajectory alongside angular velocity
comp_x_aligned = [compensated_robot.initial_x] + compensated_robot.x_list[:-1]
comp_y_aligned = [compensated_robot.initial_y] + compensated_robot.y_list[:-1]
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

#print min, max and mean position error between compensated trajectory and target trajectory
print(f"Position error between compensated trajectory and target trajectory: min {error_comp.min():.3f} m, max {error_comp.max():.3f} m, mean {error_comp.mean():.3f} m")

#plot slip estimates vs true slip value
plt.plot(slip, label='Estimated Slip')
plt.axhline(y=0.2, color='r', linestyle='--', label='True Slip Value')
plt.xlabel('Time Step')
plt.ylabel('Slip (m)')
plt.title('Estimated Slip Over Time')
plt.legend()
plt.grid()
plt.show()

# plt.plot(errors, label='Error Term')
# plt.xlabel('Time Step')
# plt.ylabel('Error Value (m)')
# plt.title('Error Term Over Time')
# plt.legend()
# plt.grid()
# plt.show()

plt.plot(estimationErrorCovarianceValues, label='Estimation Error Covariance')
plt.xlabel('Time Step')
plt.ylabel('Covariance Value')
plt.title('Estimation Error Covariance Over Time')
plt.legend()
plt.grid()
plt.show()

plt.plot(kalmanGains, label='Kalman Gain')
plt.xlabel('Time Step')
plt.ylabel('Kalman Gain Value')
plt.title('Kalman Gain Over Time')
plt.legend()
plt.grid()
plt.show()

# plt.plot(estimator.theta_diff, label='Theta Difference')
# plt.xlabel('Time Step')
# plt.ylabel('Theta Difference (radians)')
# plt.title('Theta Difference Over Time')
# plt.legend()
# plt.grid()
# plt.show()


#plot compensated velocities vs reference velocities
comp_velocities = []
for timestep in range(len(compensated_feedforward.df)):
    vel_right, vel_left = compensated_feedforward.vel_at_timestep(timestep)
    comp_velocities.append((vel_right, vel_left))   
vel_right_ref = [feedforward.vel_at_timestep(t)[0] for t in range(len(feedforward.df))]
vel_left_ref  = [feedforward.vel_at_timestep(t)[1] for t in range(len(feedforward.df))]
vel_right_comp = [vel[0] for vel in comp_velocities]
vel_left_comp  = [vel[1] for vel in comp_velocities]
plt.figure(figsize=(10, 4))
plt.subplot(1, 2, 1)
plt.plot(vel_right_ref, label='Reference right wheel velocity', linestyle='--')
plt.plot(vel_right_comp, label='Compensated right wheel velocity', linestyle='-.')
plt.xlabel('Timestep')
plt.ylabel('Velocity (m/s)')
plt.title('Right Wheel Velocity Comparison')
plt.legend()
plt.grid()
plt.subplot(1, 2, 2)
plt.plot(vel_left_ref, label='Reference left wheel velocity', linestyle='--')
plt.plot(vel_left_comp, label='Compensated left wheel velocity', linestyle='-.')
plt.xlabel('Timestep')
plt.ylabel('Velocity (m/s)')
plt.title('Left Wheel Velocity Comparison')
plt.legend()
plt.grid()
plt.tight_layout()
plt.show()