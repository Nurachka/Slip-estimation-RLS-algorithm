import numpy as np
import math
import pandas as pd
from scipy.spatial.transform import Rotation


class InterpolationOfTime:
    def __init__(self):
        pass
    def convert_unix(self, df):
        if 'unix_timestamp' not in df.columns:
            df = df.copy()
        # first, convert the time format to unix timestamp
        df.loc[:, 'unix_timestamp'] = pd.to_datetime(df['time'], format='%Y/%m/%d/%H:%M:%S.%f').astype('int64') # convert to seconds
        df = df.sort_values(by='unix_timestamp').reset_index(drop=True)       
        return df
    
    def quaternions_to_euler(self, df_imu):
        """
        Convert quaternions to Euler angles (roll, pitch, yaw).
        
        Parameters:
        df_imu : DataFrame
            DataFrame containing quaternion columns '
        
        Returns:
        DataFrame with additional columns for roll, pitch, and yaw.
        """
        assert isinstance(df_imu, pd.DataFrame), "Input must be a pandas DataFrame"
        assert all(col in df_imu.columns for col in ['.orientation.x', '.orientation.y', '.orientation.z', '.orientation.w']), "DataFrame must contain quaternion columns"
        
        # create a separate df to store quaternions from the df_imu
        df_quat = pd.DataFrame()
        df_quat['x'] = df_imu['.orientation.x']
        df_quat['y'] = df_imu['.orientation.y']
        df_quat['z'] = df_imu['.orientation.z']
        df_quat['w'] = df_imu['.orientation.w']

        rot = Rotation.from_quat(df_quat)
        rot_euler = rot.as_euler('xyz', degrees=False)
        euler_df_imu = pd.DataFrame(data=rot_euler, columns=['x', 'y', 'z'])

        # add to the original df_imu
        euler_df_imu =euler_df_imu.rename(columns={'z':'theta'})
        df_imu =df_imu.join(euler_df_imu['theta'])
        df_imu = df_imu.drop(columns=['.orientation.x', '.orientation.y', '.orientation.z', '.orientation.w'])
        return df_imu

    
