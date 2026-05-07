import sys
sys.path.append("..")
from mathematical_simulator_class.robot import Robot

robot_test = Robot()

assert robot_test.x_list == [], "Initial x_list should be empty"
assert robot_test.y_list == [], "Initial y_list should be empty"
assert robot_test.theta_list == [], "Initial theta_list should be empty"

vel_right = 1.0
vel_left = 1.0
total_steps = 5
for step in range(total_steps):
    robot_test.forward_kinematics(vel_right, vel_left)
    robot_test.add_noise()

assert len(robot_test.x_list) == total_steps, f"x_list should have {total_steps} elements"
assert len(robot_test.y_list) == total_steps, f"y_list should have {total_steps} elements"
assert len(robot_test.theta_list) == total_steps, f"theta_list should have {total_steps} elements"
assert len(robot_test.x_list_noised) == total_steps, f"x_list_noised should have {total_steps} elements"
assert len(robot_test.y_list_noised) == total_steps, f"y_list_noised should have {total_steps} elements"

print("All robot tests passed.")
print("Final position:", robot_test.x_list[-1], robot_test.y_list[-1], robot_test.theta_list[-1])
print("Final noised position:", robot_test.x_list_noised[-1], robot_test.y_list_noised[-1], robot_test.theta_list_noised[-1])
