#standard library
import os
import sys
import datetime
import numpy as np
import matplotlib.pyplot as plt
import re

xs = []
ys = []
xs_slow = []
ys_slow = []

with open("pose_file.txt", "r") as f:
    for line in f:
        match = re.search(
            r"Translation:\s*\[\s*([-0-9.]+)\s*,\s*([-0-9.]+)\s*,",
            line
        )
        if match:
            x = float(match.group(1))
            y = float(match.group(2))
            xs.append(x)
            ys.append(y)

# with open("pose_straight_slow_speed.txt", "r") as f:
    # for line in f:
    #     match = re.search(
    #         r"Translation:\s*\[\s*([-0-9.]+)\s*,\s*([-0-9.]+)\s*,",
    #         line
    #     )
    #     if match:
    #         x = float(match.group(1))
    #         y = float(match.group(2))
    #         xs_slow.append(x)
    #         ys_slow.append(y)


#plotting the trajectories from both files
plt.figure(figsize=(8, 6))
plt.plot(xs, ys, marker='o', label='Tuned parameters')
# plt.plot(xs_slow, ys_slow, marker='x', label='Slow speed')
plt.title('Robot Trajectory with Tuned Parameters')
plt.xlabel('X position (m)')
plt.ylabel('Y position (m)')
plt.axis('equal')
plt.grid()
plt.legend()
plt.show()