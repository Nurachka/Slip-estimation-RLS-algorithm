import numpy as np
import math
import pandas as pd

import matplotlib.pyplot as plt

class Analysis:
    def __init__(self):
        self.df = pd.DataFrame()
        
    def read_csv(self, file_path):
        """
        Reads a CSV file and returns a DataFrame.
        """
        df = pd.read_csv(file_path)
        self.df = df
        return df
    def clean_data_imu(self, df_imu):
        """
        Cleans the DataFrame by removing unnecessary columns in imu data.
        """
        df_imu = df_imu.drop(columns=['.header.seq', '.header.stamp.secs', '.header.stamp.nsecs', '.header.frame_id'])
        df_imu = df_imu.drop(columns=['.orientation_covariance', '.angular_velocity.x', '.angular_velocity.y', '.angular_velocity_covariance', '.linear_acceleration.x',
       '.linear_acceleration.y', '.linear_acceleration.z',
       '.linear_acceleration_covariance'])
        return df_imu
    def clean_data_cmd_vel(self, df_cmd):
        """
        Cleans the DataFrame by removing unnecessary columns in cmd vel data.
        """
        df_cmd = df_cmd.drop(columns=['.linear.x', '.linear.y', '.linear.z', '.angular.x', '.angular.y'])
        return df_cmd
            
    