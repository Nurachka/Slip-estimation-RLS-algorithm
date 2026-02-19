import numpy as np
import math
import pandas as pd

import matplotlib.pyplot as plt

class Feedforward:
    def __init__(self, df):
        self.df = df
        assert isinstance(self.df, pd.DataFrame), "Input must be a pandas DataFrame"
        assert 'right_vel' in self.df.columns, "DataFrame must contain 'right_vel' column"
        assert 'left_vel' in self.df.columns, "DataFrame must contain 'left_vel'' column"
        
    def vel_at_timestep(self, timestamp):
        if timestamp < len(self.df):
            # extract the row at the current time index
            vel_right = self.df['right_vel'].iloc[timestamp]
            vel_left = self.df['left_vel'].iloc[timestamp]
            return vel_right, vel_left
    def x_y_at_timestep(self, timestamp):
        if timestamp < len(self.df):
            # extract the row at the current time index
            x = self.df['x'].iloc[timestamp]
            y = self.df['y'].iloc[timestamp]
            
            return x, y