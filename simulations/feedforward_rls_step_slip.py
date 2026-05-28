import sys
import os
sys.path.append("..")
from mathematical_simulator_class.robot import Robot
from mathematical_simulator_class.file_reader import Analysis
from mathematical_simulator_class.feedforward import Feedforward
from mathematical_simulator_class.recursive_least_square import RecursiveLeastSquares
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

actual_state_list = []

# calculate the trajectrory with slip and noise
for timestep in range(len(feedforward.df)):
    robot.slip = 0.2 if timestep >= 200 else 0.0
    vel_right, vel_left = feedforward.vel_at_timestep(timestep)
    x,y,theta = robot.forward_kinematics(vel_right, vel_left)
    x_noised, y_noised, theta_noised = robot.add_noise()
    actual_state_list.append((x, y, theta))


x_target_list = feedforward.df['x'].tolist()
y_target_list = feedforward.df['y'].tolist()

plt.plot(x_target_list, y_target_list, label='Target trajectory', linestyle=':')
plt.xlabel('X Position (m)')
plt.ylabel('Y Position (m)')
plt.title('Target Robot trajectory')
plt.legend()
plt.gca().set_aspect('equal', adjustable='box')
plt.grid()
plt.show()


compensated_robot = Robot(initial_x=1.0, initial_y=0.0, initial_theta=1.5786512774347865)
comp_estimator = RecursiveLeastSquares(s0=np.array([0.0]), P0=10*np.eye(1,1), R=0.00436*np.eye(1,1))

theta_previous = 1.5786512774347865
compensated_state_list = []
slip_estimates = []
estimationErrorCovarianceValues = []

for timestep in range(len(feedforward.df)):
    compensated_robot.slip = 0.2 if timestep >= 200 else 0.0

    vel_right, vel_left = feedforward.vel_at_timestep(timestep)

    s_hat = np.clip(float(comp_estimator.estimates[-1][0]), -0.5, 0.5)
    if timestep > 50 and s_hat > 0.0:
        vel_right_comp = vel_right / (1 - s_hat)
        vel_left_comp  = vel_left  / (1 - s_hat)
    else:
        vel_right_comp = vel_right
        vel_left_comp  = vel_left

    x, y, theta = compensated_robot.forward_kinematics(vel_right_comp, vel_left_comp)
    x_noised, y_noised, theta_noised = compensated_robot.add_noise()

    comp_estimator.predict_sim(
        theta_noised, theta_previous, vel_right_comp, vel_left_comp, 0.05
    )
    theta_previous = theta_noised

    compensated_state_list.append(np.array([x, y, theta]))
    slip_estimates.append(float(comp_estimator.estimates[-1][0]))
    estimationErrorCovarianceValues.append(comp_estimator.estimationErrorCovarianceMatrices[timestep][0])


plt.plot([state[0] for state in compensated_state_list], [state[1] for state in compensated_state_list], label='Compensated trajectory', linestyle='-.')
plt.plot(x_target_list, y_target_list, label='Target trajectory', linestyle=':')
plt.plot([state[0] for state in actual_state_list], [state[1] for state in actual_state_list], label='Actual trajectory', linestyle='--')
plt.xlabel('X Position (m)')
plt.ylabel('Y Position (m)')
plt.title('Compensated Robot trajectory')
plt.legend()
plt.gca().set_aspect('equal', adjustable='box')
plt.grid()
plt.show()



true_slip = [0.2 if t >= 200 else 0.0 for t in range(len(feedforward.df))]
plt.plot(slip_estimates, label='Online estimated slip (compensated robot)')
plt.plot(true_slip, label='True slip', linestyle='--')
plt.xlabel('Time Step')
plt.ylabel('Slip Value')
plt.title('Estimated Slip Over Time')
plt.legend()
plt.grid()
plt.show()

# --- Tracking error ---
# state_list[k] is the state *after* step k, so prepend initial state and drop last to align with reference[k]
INITIAL = np.array([1.0, 0.0, 1.5786512774347865])
actual_arr = np.array(actual_state_list)
comp_arr   = np.array(compensated_state_list)

actual_aligned = np.vstack([INITIAL, actual_arr[:-1]])
comp_aligned   = np.vstack([INITIAL, comp_arr[:-1]])

reference_xy = np.column_stack([np.array(x_target_list), np.array(y_target_list)])

error_actual = np.linalg.norm(actual_aligned[:, :2] - reference_xy, axis=1)
error_comp   = np.linalg.norm(comp_aligned[:, :2]   - reference_xy, axis=1)

plt.figure(figsize=(10, 4))
plt.plot(error_actual, label='Actual (no compensation)', linestyle='--')
plt.plot(error_comp,   label='Compensated (RLS feedforward)')
plt.xlabel('Time Step')
plt.ylabel('Tracking Error (m)')
plt.title('Tracking Error Over Time')
plt.legend()
plt.grid()
plt.tight_layout()
plt.show()

print(f"Actual      error: min {error_actual.min():.3f} m, max {error_actual.max():.3f} m, mean {error_actual.mean():.3f} m")
print(f"Compensated error: min {error_comp.min():.3f} m,   max {error_comp.max():.3f} m,   mean {error_comp.mean():.3f} m")

#plot estimation error covariance values over time
plt.figure(figsize=(10, 4))
plt.plot(estimationErrorCovarianceValues, label='Estimation error covariance (P)', color='tab:purple')
plt.xlabel('Time Step')
plt.ylabel('Estimation Error Covariance')
plt.title('RLS Estimation Error Covariance Over Time')
plt.legend()
plt.grid()
plt.tight_layout()
plt.show()