import sys
import os
sys.path.append("..")
from mathematical_simulator.robot import Robot
from mathematical_simulator.file_reader import Analysis
from mathematical_simulator.feedforward import Feedforward
from mathematical_simulator.recursive_least_square import RecursiveLeastSquares
from mathematical_simulator.compensator import Compensator
from mathematical_simulator.interpolation import InterpolationOfTime
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

file_reader = Analysis()
robot = Robot()
compensator = Compensator()

current_dir = os.path.dirname(os.path.abspath(__file__))
trajectory_dir = os.path.join(current_dir, 'trajectories')
file_path = os.path.join(trajectory_dir, 'lemniscate_trajectory.csv')

df_trajectory = file_reader.read_csv(file_path)
print("Original velocities:", df_trajectory['right_vel'].head(5), df_trajectory['left_vel'].head(5))

#plot thetrajectory
plt.figure(figsize=(10, 5))
plt.plot(df_trajectory['x'], df_trajectory['y'], label='Original Trajectory')
plt.xlabel('X Position (m)')
plt.ylabel('Y Position (m)')
plt.title('Lemniscate gerono Trajectory')
plt.legend()
plt.axis('equal')
plt.grid()
plt.show()

df_modified = compensator.modify_velocities(0.06, df_trajectory)
print("Modified velocities:", df_modified['right_vel'].head(5), df_modified['left_vel'].head(5))

#save the modified dataframe as a csv file
#df_modified.to_csv(os.path.join(trajectory_dir, 'lemniscate_trajectory_modified_0.06.csv'), index=False)