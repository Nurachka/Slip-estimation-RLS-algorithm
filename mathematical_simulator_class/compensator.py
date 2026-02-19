import numpy as np
import pandas as pd
from copy import deepcopy
import matplotlib.pyplot as plt


class Compensator:
    def __init__(self):
        pass
    
    def modify_velocities(self, slip, df):
        """
        Modify the velocities in the DataFrame based on a slip factor.
        
        Parameters:
        slip : float
            Slip factor to adjust the velocities.
        """
        assert isinstance(df, pd.DataFrame), "Input must be a pandas DataFrame"
        assert 'right_vel' in df.columns, "DataFrame must contain 'right_vel' column"
        assert 'left_vel' in df.columns, "DataFrame must contain 'left_vel' column"
        df_copy = deepcopy(df)
        df_copy['right_vel'] /= (1 - slip)
        df_copy['left_vel'] /= (1 - slip)
        return df_copy