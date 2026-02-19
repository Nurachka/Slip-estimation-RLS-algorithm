# Mathematical Simulator

A mathematical simulation created as a part of my master  degree research in Toyohashi University of Technology for slip estimation and compensation using Recursive Least Squares (RLS) estimation. 

## Overview

This project simulates an RLS-based control system that estimates surface slip and compensates robot velocity to improve trajectory accuracy.

## Classes

### `compensator`

`modify_velocities` takes slip estimates from the RLS module and applies recalculated velocity to the robot's command signals, reducing trajectory error caused by surface friction.

### `feedforward`
Extracts data from the dataframe and returns x,y coordinates and velocities for the left and right wheel.

### `file_reader`
Reads csv file. Additionally cleans the dataframe by removing unnecessary columns.

### `interpolation`
- `convert_unix` converts the sensor type unix time to the seconds. 
- `quaternions_to_euler` converts to the roll pitch yaw 

### `recursive_least_square`
- `predict_sim` implements RLS algorithm to estimate surface slip in simulation. 
- `predict_exp` implements RLS algorithm to estimate surface slip for the real environment experiment.

### `robot`
Simulates robot dynamics and kinematic behavior in order to produce x,y,theta coordinates for the designated trajectory.

### `controllers`
- `rls_compensator.py` receives slip estimates from the RLS module and recalculates input velocities for the left and right wheels to compensate for surface slip.
- `rls_online.py` implements the same compensation strategy but applies it in real-time during robot operation.
- `feedforward.py` implements feedforward controller. This is needed just for comparing the results with other controllers to estimate the improvements. 


## Testing & Integration

All classes were individually unit tested in the `tests` folder to verify correct functionality. These tested components were then integrated to create a complete simulation in `mathematical_simulation_tests` folder to:

1. Estimates surface slip using RLS
2. Compensates velocity based on slip estimation
3. Improves robot trajectory

## Additional Folders

Other folders in the project are dedicated to plotting experiment results and localization visualization.

