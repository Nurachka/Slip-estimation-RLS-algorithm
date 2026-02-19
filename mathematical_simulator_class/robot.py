import numpy as np
import math
import pandas as pd

import matplotlib.pyplot as plt

class Robot:
    def __init__(self, initial_x=0.0, initial_y=0.0, initial_theta=0.0):
        self.wheel_base = 0.5  # Distance between the wheels in meters   
        self.timestep = 0.05 # Time step for simulation in seconds
        self.slip = 0.03  # Slip factor, a value between 0 and 1
        self.noise_std_position = 0.025  # Standard deviation for position noise
        self.noise_std_orientation = 0.0436  # Standard deviation for orientation noise
        self.initial_x = initial_x
        self.initial_y = initial_y
        self.initial_theta = initial_theta
        self.x_list = []
        self.y_list = []
        self.theta_list = []
        self.x_list_noised = []
        self.y_list_noised = []
        self.theta_list_noised = []


    def forward_kinematics(self, vel_right, vel_left):
        x_prev = self.x_list[-1] if self.x_list else self.initial_x
        y_prev = self.y_list[-1] if self.y_list else self.initial_y
        theta_prev = self.theta_list[-1] if self.theta_list else self.initial_theta

        v = (1-self.slip)*(vel_right + vel_left) / 2.0  # Linear velocity
        omega = (1-self.slip)*(vel_right - vel_left) / self.wheel_base  # Angular velocity
        # Update position and orientation
        x = v * np.cos(theta_prev) * self.timestep + x_prev
        y = v * np.sin(theta_prev) * self.timestep + y_prev
        theta = omega  * self.timestep + theta_prev
        self.x_list.append(x)
        self.y_list.append(y)  
        self.theta_list.append(theta)
        return x, y, theta
    
    def add_noise(self):
        x,y, theta = self.x_list[-1], self.y_list[-1], self.theta_list[-1]
        x += np.random.normal(0, self.noise_std_position)
        y += np.random.normal(0, self.noise_std_position)
        theta += np.random.normal(0, self.noise_std_orientation)
        self.x_list_noised.append(x)
        self.y_list_noised.append(y)    
        self.theta_list_noised.append(theta)
        return x, y, theta
    

   
