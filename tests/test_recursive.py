import sys
import os
sys.path.append("..")
import pandas as pd
import numpy as np
from mathematical_simulator.recursive_least_square import RecursiveLeastSquares


RLS = RecursiveLeastSquares(x0=np.random.rand(1,1), P0=100*np.eye(1,1),R= 0.0004*np.eye(1,1), forgettingFactor=1)

#getting the file path of the current script
current_dir = os.path.dirname(os.path.abspath(__file__))
#go one directory back to find the 'trajectories' directory
data_dir = os.path.join(current_dir, 'data')
file_path = os.path.join(data_dir, 'synced_cmdvel_imu.csv')

df = pd.read_csv(file_path)
assert 'theta_diff' in df.columns, "DataFrame must contain 'theta_diff' column"
assert 'time' in df.columns, "DataFrame must contain 'time' column"
assert 'angular_vel_z' in df.columns, "DataFrame must contain 'angular_vel_z' column"
estimated_slip  = []



for timestep in np.arange(np.size(df['time'])):
    C_matrix=np.array([0.05*df['angular_vel_z'][timestep]])
    RLS.predict(0.05*df['angular_vel_z'][timestep]-df['theta_diff'][timestep], C_matrix)

#Add the calculated by rls slip to the list
# extract the estimates in order to plot the results

for values in np.arange(np.size(df['time'])):
     estimated_slip.append(RLS.estimates[values][0])

print("Estimated slip value:", estimated_slip[-1])


