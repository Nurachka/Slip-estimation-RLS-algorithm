import sys
sys.path.append("..")
from mathematical_simulator.robot import Robot

robot_test = Robot()
""""
# Test initial position
assert robot_test.x_list == [], "Initial x_list should be empty"
assert robot_test.y_list == [], "Initial y_list should be empty"    
assert robot_test.theta_list == [], "Initial theta_list should be empty"    

# Test forward kinematics with some example velocities
vel_right = 1.0  # Example right wheel velocity in m/s
vel_left = 1.0   # Example left wheel velocity in m/s
robot_test.forward_kinematics(vel_right, vel_left)
assert len(robot_test.x_list) == 1, "x_list should contain one element after forward kinematics"
assert len(robot_test.y_list) == 1, "y_list should contain one element after forward kinematics"
assert len(robot_test.theta_list) == 1, "theta_list should contain one element after forward kinematics"
print("Updated position:", robot_test.x_list, robot_test.y_list, robot_test.theta_list)

robot_test.add_noise()
print("Noised position:", robot_test.x_list_noised, robot_test.y_list_noised, robot_test.theta_list_noised)
"""
vel_right = 1.0  # Example right wheel velocity in m/s
vel_left = 1.0   # Example left wheel velocity in m/s
#test both forward kinematics and add noise for multiple steps
total_steps = 5
for step in range(total_steps):
    robot_test.forward_kinematics(vel_right, vel_left)
    robot_test.add_noise()
    print("Position after {} steps:".format(step), robot_test.x_list, robot_test.y_list, robot_test.theta_list)
    print("Noised position after {} steps:".format(step), robot_test.x_list_noised, robot_test.y_list_noised, robot_test.theta_list_noised)

