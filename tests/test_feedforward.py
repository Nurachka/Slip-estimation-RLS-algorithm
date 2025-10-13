import sys
sys.path.append("..")
from mathematical_simulator.feedforward import Feedforward
import pandas as pd

df = pd.DataFrame({
    'right_vel': [1.0, 1.2, 1.5, 1.3, 1.1],
    'left_vel': [0.8, 0.9, 1.0, 0.7, 0.6]

})


feedforward_test = Feedforward(df)
vel_right, vel_left = feedforward_test.vel_at_timestep(1)
assert vel_right == 1.2, f"Expected 1.2, got {vel_right}"
assert vel_left == 0.9, f"Expected 0.9, got {vel_left}"
