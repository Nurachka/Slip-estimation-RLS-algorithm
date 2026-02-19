import sys
import os
sys.path.append("..")
from mathematical_simulator_class.robot import Robot
from mathematical_simulator_class.file_reader import Analysis
from mathematical_simulator_class.feedforward import Feedforward
from mathematical_simulator_class.recursive_least_square import RecursiveLeastSquares
from mathematical_simulator_class.compensator import Compensator
from mathematical_simulator_class.interpolation import InterpolationOfTime
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

file_reader = Analysis()
robot = Robot()
interpolation = InterpolationOfTime()
recursive = RecursiveLeastSquares(s0=np.eye(1,1), P0=100*np.eye(1,1), R=0.0004*np.eye(1,1))
compensator = Compensator()

#get the path for the data directory where imu and cmd vel data is stored
data_dir = os.path.join(sys.path[0], 'tests', 'data')
#file path for the cmd vel data
file_path = os.path.join(data_dir, 'gerono-cmd_vel.csv')
#file path for the imu data
file_path_imu = os.path.join(data_dir, 'gerono-imu2.csv')

#reading the cmd vel data
df_cmd = file_reader.read_csv(file_path)
#reading the imu data
df_imu = file_reader.read_csv(file_path_imu)

#removing unnecessary columns from the cmd vel data
df_cmd_filtered = file_reader.clean_data_cmd_vel(df_cmd)
#removing unnecessary columns from the imu data
df_imu_filtered = file_reader.clean_data_imu(df_imu)

# converting quaternions to euler angles in the imu data
df_imu_filtered = interpolation.quaternions_to_euler(df_imu_filtered)

#deleting the rows that are before 53.22 sec, because the cmd vel data starts at 53.22 sec
df_imu_filtered = df_imu_filtered[df_imu_filtered['time'] >= '2025/06/01/05:05:53.230896']
#deleting the rows after 2025/06/01/05:06:12.925832 because cmd vel stops sending velocity after that
df_imu_filtered = df_imu_filtered[df_imu_filtered['time'] <='2025/06/01/05:06:14.734712']
#resetting the index of the imu data
df_imu_filtered.reset_index(drop=True, inplace=True)

#converting the time column to unix timestamp in both dataframes
df_cmd_filtered = interpolation.convert_unix(df_cmd_filtered)
df_imu_filtered = interpolation.convert_unix(df_imu_filtered)

#drop the time column from the cmd vel data and imu data
df_cmd_filtered.drop(columns=['time'], inplace=True)
df_imu_filtered.drop(columns=['time'], inplace=True)

#setting the unix timestamp as index for both dataframes
df_imu_filtered.set_index('unix_timestamp', inplace=True)
df_cmd_filtered.set_index('unix_timestamp', inplace=True)

#rename the .angular.z column to nominal_angular_velocity
df_cmd_filtered.rename(columns={'.angular.z': 'nominal_angular_velocity'}, inplace=True)

#combine the two dataframes on unix timestamp
#this will align the data based on the unix timestamp
df_combined = pd.concat([df_cmd_filtered, df_imu_filtered], ignore_index=False, axis=1)
#sort the combined dataframe by unix timestamp
df_combined.sort_index(inplace=True)
#interpolate the theta column in the combined dataframe
#this is to fill the missing values in the theta column
df_combined['theta'] = df_combined['theta'].interpolate(method='index', limit_direction='both')
#unwrap theta column to avoid discontinuities
df_combined['theta_unwrapped'] = np.unwrap(df_combined['theta'].values)

#delete the rows where theta_unwrapped is less than -4 radians, cause there are 2 discontinuities in the data
#this is to remove the discontinuities in the theta_unwrapped column
df_combined = df_combined[df_combined['theta_unwrapped'] >= -4]
#reset the index of the combined dataframe
df_combined.reset_index(drop=True, inplace=True)


# now separate indices of cmd from imu by saving only rows where nominal_angular_velocity is not NaN
df_cmd_synced = df_combined[df_combined['nominal_angular_velocity'].notna()]
# reset the index of the cmd synced data
df_cmd_synced.reset_index(drop=True, inplace=True)
print("cmd synced with imu:", df_cmd_synced)

#calculate theta difference for the synced cmd vel data
df_cmd_synced['theta_diff'] = df_cmd_synced['theta_unwrapped'].diff().fillna(0)
print("Theta difference for cmd synced data:", df_cmd_synced.head(20))

slip = []
delta_t = 0.05  # time step in seconds
#applying the recursive least squares estimator to the theta difference of the synced cmd vel data
for timestep in np.arange(np.size(df_cmd_synced['theta_diff'])):
    #first claculating C matrix
    C_matrix = np.array([df_cmd_synced['nominal_angular_velocity'][timestep]* delta_t])
    recursive.predict_exp(df_cmd_synced['nominal_angular_velocity'][timestep] * delta_t - df_cmd_synced['theta_diff'][timestep], C_matrix)
    slip.append(recursive.estimates[timestep][0])
# print the estimated slip values
print("Estimated slip values:", slip[-1])

# Plotting the slip values
plt.plot(slip, label='Estimated Slip')
plt.xlabel('Time Step')
plt.ylabel('Slip Value')
plt.title('Estimated Slip Over Time')
plt.legend()
plt.grid()
plt.show()


#file path for lemniscate_trajectory.csv
current_dir = os.path.dirname(os.path.abspath(__file__))
trajectory_dir = os.path.join(current_dir, 'trajectories')
file_path = os.path.join(trajectory_dir, 'lemniscate_trajectory.csv')

df_trajectory = file_reader.read_csv(file_path)
#print("Original velocities:", df_trajectory['right_vel'].head(5), df_trajectory['left_vel'].head(5))

df_modified = compensator.modify_velocities(slip[-1], df_trajectory)
#print("Modified velocities:", df_modified['right_vel'].head(5), df_modified['left_vel'].head(5))

#slip 0.1 df 
df_modified_0_1 = compensator.modify_velocities(0.1, df_trajectory)
print("Modified velocities with slip 0.1:", df_modified_0_1['right_vel'].head(5), df_modified_0_1['left_vel'].head(5))
#save the modified dataframe as a csv file
df_modified_0_1.to_csv(os.path.join(trajectory_dir, 'lemniscate_trajectory_modified_0.1.csv'), index=False)



#slip 0.15 df 
df_modified_0_15 = compensator.modify_velocities(0.15, df_trajectory)
print("Modified velocities with slip 0.15:", df_modified_0_15['right_vel'].head(5), df_modified_0_15['left_vel'].head(5))
#save the modified dataframe as a csv file
df_modified_0_15.to_csv(os.path.join(trajectory_dir, 'lemniscate_trajectory_modified_0.15.csv'), index=False)


#slip 0.2 df 
df_modified_0_2 = compensator.modify_velocities(0.2, df_trajectory)
print("Modified velocities with slip 0.2:", df_modified_0_2['right_vel'].head(5), df_modified_0_2['left_vel'].head(5))
#save the modified dataframe as a csv file
df_modified_0_2.to_csv(os.path.join(trajectory_dir, 'lemniscate_trajectory_modified_0.2.csv'), index=False)

#slip 0.26 df 
df_modified_0_26 = compensator.modify_velocities(0.26, df_trajectory)
print("Modified velocities with slip 0.26:", df_modified_0_26['right_vel'].head(5), df_modified_0_26['left_vel'].head(5))
#save the modified dataframe as a csv file
df_modified_0_26.to_csv(os.path.join(trajectory_dir, 'lemniscate_trajectory_modified_0.26.csv'), index=False)





