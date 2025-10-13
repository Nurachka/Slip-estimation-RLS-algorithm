import pandas as pd
from scipy.spatial.transform import Rotation
import math
import matplotlib.pyplot as plt
import numpy as np
import sys
import os
sys.path.append("..")


current_dir = sys.path[0]
file_path_cmd = f"{current_dir}/gerono-cmd_vel.csv"
df_cmd = pd.read_csv(file_path_cmd)
file_path_imu = f"{current_dir}/gerono-imu2.csv"
df_imu = pd.read_csv(file_path_imu)


