import sys
sys.path.append("..")
from mathematical_simulator.interpolation import InterpolationOfTime
from mathematical_simulator.file_reader import Analysis
import pandas as pd
import os
import numpy as np
import matplotlib.pyplot as plt

analysis = Analysis()
interpolation = InterpolationOfTime()

#the path to the vel data file
current_dir = sys.path[0]
data_dir = f"{current_dir}/data"
file_path = f"{data_dir}/gerono-cmd_vel.csv"
df_cmd = analysis.read_csv(file_path)


#the path to the imu data file
file_path_imu = f"{data_dir}/gerono-imu2.csv"
df_imu = analysis.read_csv(file_path_imu)

df_imu = interpolation.quaternions_to_euler(df_imu)

#deleting the rows that are before 53.22 sec, because the cmd vel data starts at 53.22 sec
df_imu = df_imu[df_imu['time'] >= '2025/06/01/05:05:53.230896']
#deleting the rows after 2025/06/01/05:06:12.925832 because cmd vel stops sending velocity after that
df_imu_filtered = df_imu[df_imu['time'] <='2025/06/01/05:06:14.734712']
#resetting the index of the imu data
df_imu_filtered.reset_index(drop=True, inplace=True)

#deleting rows where ang velocity is 0 in cmd vel data
df_cmd_filtered = df_cmd[df_cmd['.angular.z'] != 0]


#converting the time column to unix timestamp in both dataframes
df_cmd_filtered = interpolation.convert_unix(df_cmd_filtered)
df_imu_filtered = interpolation.convert_unix(df_imu_filtered)

#drop the time column from the cmd vel data and imu data
df_cmd_filtered.drop(columns=['time'], inplace=True)
df_imu_filtered.drop(columns=['time'], inplace=True)


#set the unix timestamp as index for both dataframes
df_imu_filtered.set_index('unix_timestamp', inplace=True)
df_cmd_filtered.set_index('unix_timestamp', inplace=True)

#also dropping unnecessary columns from the cmd vel data
df_cmd_filtered = df_cmd_filtered.drop(['.linear.x', '.linear.y', '.linear.z', '.angular.x', '.angular.y'], axis=1)

 
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

### BECAUSE THE UNRAPPED DATA HAS 2 DISCONTINUTINES, THE INDEXES OF THE DISCONTINUTINES ARE
#print the values of theta more than -4 radians
#print("Theta values greater than -4 radians:")
#print(df_combined[df_combined['theta_unwrapped'] < -4]['theta_unwrapped'])

### NEXT THE ANGULAR VELOCITY Z FROM THE CMD VEL DATA WILL BE INTERPOLATED
#interpolate the angular velocity z column in the combined dataframe
df_combined['.angular.z'] = df_combined['.angular.z'].interpolate(method='index', limit_direction='both')


#interlolating angular velocity from imu data to check if it is the same as the angular velocity from cmd vel data
df_combined['.angular_velocity_z'] = df_combined['.angular_velocity.z'].interpolate(method='index', limit_direction='both')
print(df_combined[['.angular.z', '.angular_velocity_z']].head(20))

#plot the angular velocity z from cmd vel data and imu data
plt.figure(figsize=(10, 5))
plt.plot(df_combined.index, df_combined['.angular.z'], label='.angular.z from cmd vel', color='blue')
plt.plot(df_combined.index, df_combined['.angular_velocity_z'], label='.angular_velocity.z from imu', color='orange')
plt.title('Angular Velocity Z from Cmd Vel and IMU Over Time')
plt.xlabel('Unix Timestamp')
plt.ylabel('Angular Velocity Z (rad/s)')
plt.legend()
plt.grid()
plt.show()