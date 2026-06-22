import sys
import os
sys.path.append("..")
from mathematical_simulator_class.robot import Robot
from mathematical_simulator_class.file_reader import Analysis
from mathematical_simulator_class.feedforward import Feedforward
from mathematical_simulator_class.recursive_least_square import RecursiveLeastSquares
from mathematical_simulator_class.config import WHEEL_BASE, TIMESTEP, NOISE_STD_ORIENTATION
import matplotlib.pyplot as plt
import numpy as np

###Online implementation of the RLS algorithm with compensator and feedforward control###

# at t=0 the robot is at (1, 0) with theta ≈ π/2 so the lemniscate is horizontal
INITIAL_X     = 1.0
INITIAL_Y     = 0.0
INITIAL_THETA = 1.5786512774347865


def run_simulation(feedforward):
    robot        = Robot(initial_x=INITIAL_X, initial_y=INITIAL_Y, initial_theta=INITIAL_THETA)
    robot_no_comp = Robot(initial_x=INITIAL_X, initial_y=INITIAL_Y, initial_theta=INITIAL_THETA)
    estimator    = RecursiveLeastSquares(s0=np.array([0.0]), P0=10*np.eye(1),
                                         R=2*NOISE_STD_ORIENTATION**2*np.eye(1))

    slip = []
    comp_trajectory      = []
    vel_right_list       = []
    vel_left_list        = []
    vel_right_comp_list  = []
    vel_left_comp_list   = []

    theta_previous_noised = INITIAL_THETA
    slip_previous = 0.0

    for timestep in range(len(feedforward.df)):
        vel_right, vel_left = feedforward.vel_at_timestep(timestep)
        vel_right_list.append(vel_right)
        vel_left_list.append(vel_left)

        slip_clamped  = max(min(slip_previous, 0.2), 0.0)
        vel_right_comp = vel_right / (1 - slip_clamped)
        vel_left_comp  = vel_left  / (1 - slip_clamped)
        vel_right_comp_list.append(vel_right_comp)
        vel_left_comp_list.append(vel_left_comp)

        x_a, y_a, theta_a = robot.forward_kinematics(vel_right_comp, vel_left_comp)
        x_noised, y_noised, theta_noised = robot.add_noise()
        robot_no_comp.forward_kinematics(vel_right, vel_left)

        estimator.predict_sim(theta_noised, theta_previous_noised,
                              vel_right_comp, vel_left_comp, TIMESTEP)
        theta_previous_noised = theta_noised

        comp_trajectory.append((x_a, y_a, theta_a))
        slip_previous = estimator.estimates[-1][0]
        slip.append(slip_previous)

    return robot_no_comp, comp_trajectory, slip, vel_right_list, vel_left_list, vel_right_comp_list, vel_left_comp_list


if __name__ == "__main__":
    file_reader    = Analysis()
    current_dir    = os.path.dirname(os.path.abspath(__file__))
    trajectory_dir = os.path.join(current_dir, '..', 'trajectories')
    file_path      = os.path.join(trajectory_dir, 'lemniscate_trajectory.csv')
    feedforward    = Feedforward(file_reader.read_csv(file_path))

    robot_no_comp, comp_trajectory, slip, \
        vel_right_list, vel_left_list, vel_right_comp_list, vel_left_comp_list = run_simulation(feedforward)

    x_target_list     = feedforward.df['x'].tolist()
    y_target_list     = feedforward.df['y'].tolist()
    theta_target_list = feedforward.df['theta'].tolist()

    print("Estimated slip values:", slip[-1])

    plt.plot(slip, label='Estimated Slip')
    plt.xlabel('Time Step')
    plt.ylabel('Slip')
    plt.title('Estimated Slip Over Time')
    plt.legend()
    plt.grid()
    plt.show()

    plt.plot(x_target_list, y_target_list, label='Reference trajectory', linestyle=':')
    plt.plot(robot_no_comp.x_list, robot_no_comp.y_list, label='Robot Path without compensation', linestyle=':')
    plt.plot([x for x, y, theta in comp_trajectory], [y for x, y, theta in comp_trajectory],
             label='Compensated trajectory', linestyle='--')
    plt.xlabel('X Position (m)')
    plt.ylabel('Y Position (m)')
    plt.title('Reference vs Compensated Trajectory')
    plt.legend()
    plt.axis('equal')
    plt.grid()
    plt.show()

    comp_x_aligned     = [INITIAL_X]     + [x     for x, y, theta in comp_trajectory[:-1]]
    comp_y_aligned     = [INITIAL_Y]     + [y     for x, y, theta in comp_trajectory[:-1]]
    comp_theta_aligned = [INITIAL_THETA] + [theta for _, _, theta  in comp_trajectory[:-1]]
    error_comp    = np.linalg.norm(
        np.column_stack((x_target_list, y_target_list)) -
        np.column_stack((comp_x_aligned, comp_y_aligned)), axis=1)
    heading_error = np.abs(np.arctan2(
        np.sin(np.array(theta_target_list) - np.array(comp_theta_aligned)),
        np.cos(np.array(theta_target_list) - np.array(comp_theta_aligned))))

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
    ax1.plot(vel_right_list, label='Nominal', linestyle='--')
    ax1.plot(vel_right_comp_list, label='Compensated')
    ax1.set_ylabel('Velocity (m/s)')
    ax1.set_title('Right Wheel Velocity')
    ax1.legend()
    ax1.grid()
    ax2.plot(vel_left_list, label='Nominal', linestyle='--')
    ax2.plot(vel_left_comp_list, label='Compensated')
    ax2.set_xlabel('Time Step')
    ax2.set_ylabel('Velocity (m/s)')
    ax2.set_title('Left Wheel Velocity')
    ax2.legend()
    ax2.grid()
    plt.tight_layout()
    plt.show()

    angular_velocity = [(vr - vl) / WHEEL_BASE
                        for vr, vl in zip(vel_right_comp_list, vel_left_comp_list)]
    plt.plot(angular_velocity, label='Angular Velocity (compensated)')
    plt.xlabel('Time Step')
    plt.ylabel('Angular Velocity (rad/s)')
    plt.title('Angular Velocity Over Time')
    plt.legend()
    plt.grid()
    plt.show()

    print(f"Position error  — mean: {error_comp.mean():.4f} m,  max: {error_comp.max():.4f} m")
    print(f"Heading error   — mean: {np.degrees(heading_error.mean()):.4f} deg, max: {np.degrees(heading_error.max()):.4f} deg")
    print(f"Position error  — RMSE: {np.sqrt(np.mean(error_comp**2)):.4f} m")
    print(f"Heading error   — RMSE: {np.degrees(np.sqrt(np.mean(heading_error**2))):.4f} deg")
    print(f"Position error  — std dev: {error_comp.std():.4f} m")
    print(f"Heading error   — std dev: {np.degrees(heading_error.std()):.4f} deg")

    plt.figure(figsize=(12, 5))
    plt.subplot(1, 2, 1)
    plt.plot(error_comp, label='Position Error (compensated vs target)')
    plt.xlabel('Time Step')
    plt.ylabel('Error (m)')
    plt.title('Position Error Over Time')
    plt.legend()
    plt.grid()
    plt.subplot(1, 2, 2)
    plt.plot(np.degrees(heading_error), label='Heading Error (compensated vs target in degrees)')
    plt.xlabel('Time Step')
    plt.ylabel('Error (degrees)')
    plt.title('Heading Error Over Time')
    plt.legend()
    plt.grid()
    plt.tight_layout()
    plt.show()
