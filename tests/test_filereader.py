import os
import sys
sys.path.append("..")
from mathematical_simulator.file_reader import Analysis

file_reader_test = Analysis()
current_dir = os.path.dirname(os.path.abspath(__file__))
print("Current directory:", current_dir)
data_dir = os.path.join(current_dir, 'data')
print("Data directory:", data_dir)
file_path = os.path.join(data_dir, 'robot_velocity_data.csv')
print("File path:", file_path)
df = file_reader_test.read_csv(file_path)
assert not df.empty, "DataFrame should not be empty after reading CSV"
assert 'right_vel' in df.columns, "DataFrame must contain 'right_vel' column"
assert 'left_vel' in df.columns, "DataFrame must contain 'left_vel' column"
print("DataFrame read successfully with columns:", df.columns.tolist()) 


