import sys
sys.path.append("..")
from mathematical_simulator.compensator import Compensator
from mathematical_simulator.file_reader import Analysis
import pandas as pd
import os
analysis = Analysis()
compensator = Compensator()

current_dir = sys.path[0]
data_dir = f"{current_dir}/data"
file_path = f"{data_dir}/lemniscate_trajectory.csv"
df = analysis.read_csv(file_path)
print("Original DataFrame:", df)
modified_df = compensator.modify_velocities(0.3, df)
print("Modified DataFrame:", df)