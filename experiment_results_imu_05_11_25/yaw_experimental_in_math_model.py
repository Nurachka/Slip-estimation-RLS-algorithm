import re
import matplotlib.pyplot as plt

import sys
import os
sys.path.append("..")
from mathematical_simulator_class.robot import Robot
from mathematical_simulator_class.file_reader import Analysis
from mathematical_simulator_class.feedforward import Feedforward
from mathematical_simulator_class.recursive_least_square import RecursiveLeastSquares
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

# === Change this to your log file name ===
log_file = "experiment_inside_05.11.25"

# Regex patterns
pattern_yaw = re.compile(r"Yaw:\s*([-+]?\d*\.\d+|\d+)")
pattern_yaw_prev = re.compile(r"Yaw previous:\s*([-+]?\d*\.\d+|\d+|None)")
pattern_ang_vel = re.compile(r"Ground angular velocity z:\s*([-+]?\d*\.\d+|\d+)")
pattern_slip = re.compile(r"slip:\s*([-+]?\d*\.\d+|\d+)")

# Lists to store values
yaw_list = []
yaw_prev_list = []
ang_vel_list = []
slip_list = []

with open(log_file, "r") as f:
    for line in f:
        if "Yaw:" in line and "Ground angular velocity z" in line:
            yaw_match = pattern_yaw.search(line)
            yaw_prev_match = pattern_yaw_prev.search(line)
            ang_vel_match = pattern_ang_vel.search(line)
            if yaw_match and yaw_prev_match and ang_vel_match:
                yaw = float(yaw_match.group(1))
                yaw_prev_str = yaw_prev_match.group(1)
                yaw_prev = float(yaw_prev_str) if yaw_prev_str != "None" else None
                ang_vel = float(ang_vel_match.group(1))
                yaw_list.append(yaw)
                yaw_prev_list.append(yaw_prev)
                ang_vel_list.append(ang_vel)

        if "slip:" in line:
            slip_match = pattern_slip.search(line)
            if slip_match:
                slip = float(slip_match.group(1))
                slip_list.append(slip)


### Putting yaw and yaw previous values into the math model 


file_reader = Analysis()

current_dir = os.path.dirname(os.path.abspath(__file__))
trajectory_dir = os.path.join(current_dir, 'trajectories' )
file_path = os.path.join(trajectory_dir, 'lemniscate_trajectory.csv')

feedforward = Feedforward(file_reader.read_csv(file_path))
estimator = RecursiveLeastSquares(s0=np.array([0.0]), P0=100*np.eye(1), R=1*np.eye(1,1))
slip = []
theta_previous_noised =[]
theta_noised =[]



for timestep in range(len(feedforward.df)):
    vel_right, vel_left = feedforward.vel_at_timestep(timestep)

    theta_noised.append(yaw_list[timestep])
    theta_previous_noised.append(yaw_prev_list[timestep])
    # Calculate the slip using the recursive least squares estimator
    estimator.predict_sim(theta_noised[timestep], theta_previous_noised[timestep], vel_right, vel_left, 0.05)
    theta_previous_noised = theta_noised
    
    slip.append(estimator.estimates[timestep][0])


#extracting the x,y,theta values from the feedforward dataframe for comparison
for timestep in range(len(feedforward.df)):
    x_target, y_target= feedforward.x_y_at_timestep(timestep)
    x_target_list = feedforward.df['x'].tolist()
    y_target_list = feedforward.df['y'].tolist()


plt.plot(x_target_list, y_target_list, label='Target Path', linestyle=':')
plt.xlabel('X Position (m)')
plt.ylabel('Y Position (m)')
plt.title('Target Robot Path')  
plt.legend()
plt.axis('equal')
plt.grid()
plt.show()

print("Estimated slip values from IMU yaw data:", slip[-1])
#plot the slip values over time
plt.plot(slip, label='Estimated Slip from IMU Yaw data')
plt.xlabel('Time Step')
plt.ylabel('Slip Value')
plt.title('Estimated Slip Over Time from IMU Yaw data')
plt.legend()
plt.grid()
plt.show()

