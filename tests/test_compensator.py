import sys
sys.path.append("..")
from mathematical_simulator_class.compensator import Compensator
from mathematical_simulator_class.file_reader import Analysis
import pandas as pd
import os
analysis = Analysis()
compensator = Compensator()

current_dir = os.path.dirname(os.path.abspath(__file__))
data_dir = os.path.join(current_dir, 'data')
file_path = os.path.join(data_dir, 'lemniscate_trajectory.csv')
df = analysis.read_csv(file_path)
print("Original DataFrame:", df)
modified_df = compensator.modify_velocities(0.3, df)
print("Modified DataFrame:", modified_df)