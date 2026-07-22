import sys
import os
sys.path.append("..")
from mathematical_simulator_class.robot import Robot
from mathematical_simulator_class.file_reader import Analysis
from mathematical_simulator_class.feedforward import Feedforward
from mathematical_simulator_class.recursive_least_square import RecursiveLeastSquares
from mathematical_simulator_class.config import WHEEL_BASE, TIMESTEP, NOISE_STD_POSITION, NOISE_STD_ORIENTATION
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

###Online implementation of the RLS algorithm with compensator and feedforward control###

# at t=0 the robot is at (1, 0) with theta ≈ π/2 so the lemniscate is horizontal
INITIAL_X     = 1.0
INITIAL_Y     = 0.0
INITIAL_THETA = 1.5786512774347865
SEED          = 42  # RNG seed so measurement noise matches the MPC tests


def run_simulation(feedforward, seed=SEED):
    np.random.seed(seed)  # seed RNG so add_noise() draws match test_linear_mpc_lemniscate.py
    robot        = Robot(initial_x=INITIAL_X, initial_y=INITIAL_Y, initial_theta=INITIAL_THETA)
    robot_no_comp = Robot(initial_x=INITIAL_X, initial_y=INITIAL_Y, initial_theta=INITIAL_THETA)
    estimator    = RecursiveLeastSquares(s0=np.array([0.0]), P0=50*np.eye(1),
                                         R=2*NOISE_STD_ORIENTATION**2*np.eye(1))

    slip = []
    comp_trajectory      = []
    no_comp_trajectory   = []
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

        slip_clamped  = max(min(slip_previous, 0.3), 0.0)
        vel_right_comp = vel_right / (1 - slip_clamped)
        vel_left_comp  = vel_left  / (1 - slip_clamped)
        vel_right_comp_list.append(vel_right_comp)
        vel_left_comp_list.append(vel_left_comp)

        x_a, y_a, theta_a = robot.forward_kinematics(vel_right_comp, vel_left_comp)
        x_noised, y_noised, theta_noised = robot.add_noise()
        x_nc, y_nc, theta_nc = robot_no_comp.forward_kinematics(vel_right, vel_left)
        no_comp_trajectory.append((x_nc, y_nc, theta_nc))

        estimator.predict_sim(theta_noised, theta_previous_noised,
                              vel_right_comp, vel_left_comp, TIMESTEP)
        theta_previous_noised = theta_noised

        comp_trajectory.append((x_a, y_a, theta_a))
        slip_previous = estimator.estimates[-1][0]
        slip.append(slip_previous)

    return robot, robot_no_comp, comp_trajectory, no_comp_trajectory, slip, vel_right_list, vel_left_list, vel_right_comp_list, vel_left_comp_list


if __name__ == "__main__":
    file_reader    = Analysis()
    current_dir    = os.path.dirname(os.path.abspath(__file__))
    trajectory_dir = os.path.join(current_dir, '..', 'trajectories')
    file_path      = os.path.join(trajectory_dir, 'lemniscate_trajectory.csv')
    feedforward    = Feedforward(file_reader.read_csv(file_path))

    robot, robot_no_comp, comp_trajectory, no_comp_trajectory, slip, \
        vel_right_list, vel_left_list, vel_right_comp_list, vel_left_comp_list = run_simulation(feedforward)

    x_target_list     = feedforward.df['x'].tolist()
    y_target_list     = feedforward.df['y'].tolist()
    theta_target_list = feedforward.df['theta'].tolist()
    time_list         = feedforward.df['time'].tolist()  # seconds

    print("Estimated slip values:", slip[-1])

    # Save the compensated trajectory in the same format as lemniscate_trajectory.csv
    robot_slip     = robot_no_comp.slip
    experiments_dir = os.path.join(current_dir, '..', 'files_for_experiments')
    os.makedirs(experiments_dir, exist_ok=True)
    output_path = os.path.join(experiments_dir, f'lemniscate_compensated_slip{robot_slip}.csv')
    compensated_df = pd.DataFrame({
        'time':      feedforward.df['time'].tolist(),
        'x':         [x     for x, y, theta in comp_trajectory],
        'y':         [y     for x, y, theta in comp_trajectory],
        'theta':     [theta for x, y, theta in comp_trajectory],
        'left_vel':  vel_left_comp_list,
        'right_vel': vel_right_comp_list,
    })
    compensated_df.to_csv(output_path, index=False)
    print(f"Saved compensated trajectory to {output_path}")

    plt.plot(time_list, slip, label='Estimated Slip')
    plt.axhline(y=robot_no_comp.slip, color='r', linestyle='--', label='True Slip')
    plt.xlabel('Time (s)')
    plt.ylabel('Slip')
    plt.title('Estimated Slip Over Time')
    plt.legend()
    plt.grid()
    plt.show()

    # Time after which the estimated slip stays within THRESHOLD of the true value
    THRESHOLD = 0.01
    true_slip = robot_no_comp.slip
    within = np.abs(np.array(slip) - true_slip) <= THRESHOLD
    if not within.any():
        print(f"Estimated slip never converged within {THRESHOLD} of true slip {true_slip}")
    else:
        # first index from which the estimate stays within the threshold for the rest of the run
        conv_idx = np.where(~within)[0][-1] + 1 if (~within).any() else 0
        if conv_idx < len(slip):
            print(f"Estimated slip converged within {THRESHOLD} of true slip {true_slip} "
                  f"at t = {time_list[conv_idx]:.2f} s (step {conv_idx})")
        else:
            print(f"Estimated slip never settled within {THRESHOLD} of true slip {true_slip}")

    # reference trajectory with Gaussian position measurement noise
    x_ref_noised = np.array(x_target_list) + np.random.normal(0, NOISE_STD_POSITION, len(x_target_list))
    y_ref_noised = np.array(y_target_list) + np.random.normal(0, NOISE_STD_POSITION, len(y_target_list))

    plt.plot(x_target_list, y_target_list,  color='blue')
    plt.plot(x_ref_noised, y_ref_noised,
             color='blue', alpha=0.4, linewidth=0.8)
    plt.plot([x for x, y, theta in comp_trajectory], [y for x, y, theta in comp_trajectory],
              color='purple', )
    plt.xlabel('X Position (m)')
    plt.ylabel('Y Position (m)')
    plt.legend()
    plt.axis('equal')
    plt.grid()
    plt.show()

    # Actual trajectory with slip (nominal velocities, no compensation)
    plt.plot(robot_no_comp.x_list, robot_no_comp.y_list, label='Actual trajectory (with slip)', color='orange')
    plt.xlabel('X Position (m)')
    plt.ylabel('Y Position (m)')
    plt.title('Actual Robot Trajectory with Slip')
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

    no_comp_x_aligned     = [INITIAL_X]     + [x     for x, y, theta in no_comp_trajectory[:-1]]
    no_comp_y_aligned     = [INITIAL_Y]     + [y     for x, y, theta in no_comp_trajectory[:-1]]
    no_comp_theta_aligned = [INITIAL_THETA] + [theta for _, _, theta  in no_comp_trajectory[:-1]]
    error_no_comp = np.linalg.norm(
        np.column_stack((x_target_list, y_target_list)) -
        np.column_stack((no_comp_x_aligned, no_comp_y_aligned)), axis=1)
    heading_error_no_comp = np.abs(np.arctan2(
        np.sin(np.array(theta_target_list) - np.array(no_comp_theta_aligned)),
        np.cos(np.array(theta_target_list) - np.array(no_comp_theta_aligned))))

    plt.figure(figsize=(10, 4))
    plt.plot(time_list, vel_right_list, label='Nominal', linestyle='--')
    plt.plot(time_list, vel_right_comp_list, label='Compensated')
    plt.xlabel('Time (s)')
    plt.ylabel('Velocity (m/s)')
    plt.legend()
    plt.grid()
    plt.tight_layout()
    plt.show()

    plt.figure(figsize=(10, 4))
    plt.plot(time_list, vel_left_list, label='Nominal', linestyle='--')
    plt.plot(time_list, vel_left_comp_list, label='Compensated')
    plt.xlabel('Time (s)')
    plt.ylabel('Velocity (m/s)')
    plt.legend()
    plt.grid()
    plt.tight_layout()
    plt.show()

    angular_velocity = [(vr - vl) / WHEEL_BASE
                        for vr, vl in zip(vel_right_comp_list, vel_left_comp_list)]
    plt.plot(time_list, angular_velocity, label='Angular Velocity (compensated)')
    plt.xlabel('Time (s)')
    plt.ylabel('Angular Velocity (rad/s)')
    plt.title('Angular Velocity Over Time')
    plt.legend()
    plt.grid()
    plt.show()

    # print("--- Compensated (feedforward + RLS) ---")
    # print(f"Position error  — mean: {error_comp.mean():.4f} m,  max: {error_comp.max():.4f} m")
    # print(f"Heading error   — mean: {heading_error.mean():.4f} rad, max: {heading_error.max():.4f} rad")
    # print(f"Position error  — RMSE: {np.sqrt(np.mean(error_comp**2)):.4f} m")
    # print(f"Heading error   — RMSE: {np.sqrt(np.mean(heading_error**2)):.4f} rad")
    # print(f"Position error  — std dev: {error_comp.std():.4f} m")
    # print(f"Heading error   — std dev: {heading_error.std():.4f} rad")

    # print("--- Feedforward only (no RLS compensation) ---")
    # print(f"Position error  — mean: {error_no_comp.mean():.4f} m,  max: {error_no_comp.max():.4f} m")
    # print(f"Heading error   — mean: {heading_error_no_comp.mean():.4f} rad, max: {heading_error_no_comp.max():.4f} rad")
    # print(f"Position error  — RMSE: {np.sqrt(np.mean(error_no_comp**2)):.4f} m")
    # print(f"Heading error   — RMSE: {np.sqrt(np.mean(heading_error_no_comp**2)):.4f} rad")
    # print(f"Position error  — std dev: {error_no_comp.std():.4f} m")
    # print(f"Heading error   — std dev: {heading_error_no_comp.std():.4f} rad")

    # Position error (separate figure so it can be saved on its own)
    plt.figure(figsize=(7, 4))
    plt.plot(time_list, error_comp, color='tab:blue')
    plt.xlabel('Time (s)')
    plt.ylabel('Error in position (m)')
    plt.legend()
    plt.grid()
    plt.tight_layout()
    plt.show()

    # Heading error (separate figure so it can be saved on its own)
    plt.figure(figsize=(7, 4))
    plt.plot(time_list, heading_error, color='tab:blue')
    plt.xlabel('Time (s)')
    plt.ylabel('Error in heading (rad)')
    plt.legend()
    plt.grid()
    plt.tight_layout()
    plt.show()

    # ----- Tracking error: noisy MEASURED actual pose (RLS-compensated) vs reference -----
    # Uses the Gaussian-noised measurements collected in the compensated robot
    # (robot.*_list_noised), aligned with the same one-step shift as error_comp above.
    meas_x_aligned     = [INITIAL_X]     + robot.x_list_noised[:-1]
    meas_y_aligned     = [INITIAL_Y]     + robot.y_list_noised[:-1]
    meas_theta_aligned = [INITIAL_THETA] + robot.theta_list_noised[:-1]
    error_meas = np.linalg.norm(
        np.column_stack((x_target_list, y_target_list)) -
        np.column_stack((meas_x_aligned, meas_y_aligned)), axis=1)
    heading_error_meas = np.abs(np.arctan2(
        np.sin(np.array(theta_target_list) - np.array(meas_theta_aligned)),
        np.cos(np.array(theta_target_list) - np.array(meas_theta_aligned))))

    print("--- Measured (noisy) actual pose with RLS vs reference ---")
    print(f"Position error  — mean: {error_meas.mean():.4f} m,  max: {error_meas.max():.4f} m")
    print(f"Heading error   — mean: {heading_error_meas.mean():.4f} rad, max: {heading_error_meas.max():.4f} rad")
    print(f"Position error  — RMSE: {np.sqrt(np.mean(error_meas**2)):.4f} m")
    print(f"Heading error   — RMSE: {np.sqrt(np.mean(heading_error_meas**2)):.4f} rad")

    # Measured position tracking error (separate figure so it can be saved on its own)
    plt.figure(figsize=(7, 4))
    plt.plot(time_list, error_meas, color='blue')
    plt.xlabel('Time (s)')
    plt.ylabel('Error in position (m)')
    plt.legend()
    plt.grid()
    plt.tight_layout()
    plt.show()

    # Measured heading tracking error (separate figure so it can be saved on its own)
    plt.figure(figsize=(7, 4))
    plt.plot(time_list, heading_error_meas, color='blue')
    plt.xlabel('Time (s)')
    plt.ylabel('Error in heading (rad)')
    plt.legend()
    plt.grid()
    plt.tight_layout()
    plt.show()

    # ----- Cross-track error (Method 2: perpendicular distance to reference polyline) -----
    def point_seg_distance(px, py, ax, ay, bx, by):
        """Perpendicular distance from P=(px,py) to segment AB, plus the projected
        point, the segment tangent, and the projection fraction t. t is clamped so
        we measure to the segment, not to the infinite line."""
        abx, aby = bx - ax, by - ay
        denom = abx * abx + aby * aby
        t = 0.0 if denom == 0.0 else ((px - ax) * abx + (py - ay) * aby) / denom
        t = max(0.0, min(1.0, t))
        projx, projy = ax + t * abx, ay + t * aby
        return np.hypot(px - projx, py - projy), projx, projy, abx, aby, t

    def nearest_ref_match(px, py, i, x_ref, y_ref, window):
        """Nearest point on the reference polyline to P, searching only segments
        within +/- window of index i (index seed) so the lemniscate self-crossing
        can't snap to the other lobe. Returns the signed cross-track error
        (+ left of the path tangent, - right), the nearest segment index j, and
        the projection fraction t along that segment."""
        lo = max(0, i - window)
        hi = min(len(x_ref) - 1, i + window)  # last valid segment start
        best_d, best_signed, best_j, best_t = np.inf, np.inf, lo, 0.0
        for j in range(lo, hi):
            d, projx, projy, tx, ty, t = point_seg_distance(
                px, py, x_ref[j], y_ref[j], x_ref[j + 1], y_ref[j + 1])
            if d < best_d:
                best_d = d
                cross = tx * (py - projy) - ty * (px - projx)
                best_signed = np.sign(cross) * d
                best_j, best_t = j, t
        return best_signed, best_j, best_t

    def wrap(a):
        """Wrap angle to (-pi, pi]."""
        return np.arctan2(np.sin(a), np.cos(a))

    XTE_WINDOW = 20  # reference samples (~1.0 s at 20 Hz) around the index seed
    xte_comp   = np.empty(len(comp_x_aligned))
    xte_theta  = np.empty(len(comp_x_aligned))  # orientation cross-track error (rad)
    for i, (px, py, pth) in enumerate(zip(comp_x_aligned, comp_y_aligned, comp_theta_aligned)):
        signed, j, t = nearest_ref_match(px, py, i, x_target_list, y_target_list, XTE_WINDOW)
        xte_comp[i] = signed
        # reference heading at the nearest point: interpolate theta[j]..theta[j+1]
        # by the projection fraction t, wrapping the increment to stay on the circle.
        theta_ref_near = theta_target_list[j] + t * wrap(theta_target_list[j + 1] - theta_target_list[j])
        xte_theta[i] = wrap(pth - theta_ref_near)  # signed: + robot rotated left of path

    print("--- Cross-track error (reference vs compensated) ---")
    print(f"XTE position — mean |e|: {np.abs(xte_comp).mean():.4f} m")
    print(f"XTE position — max  |e|: {np.abs(xte_comp).max():.4f} m")
    print(f"XTE position — RMSE   : {np.sqrt(np.mean(xte_comp**2)):.4f} m")
    print(f"XTE heading  — mean |e|: {np.abs(xte_theta).mean():.4f} rad")
    print(f"XTE heading  — max  |e|: {np.abs(xte_theta).max():.4f} rad")
    print(f"XTE heading  — RMSE   : {np.sqrt(np.mean(xte_theta**2)):.4f} rad")

    # sanity check: perpendicular (cross-track) error must not exceed the
    # time-matched point-to-point distance; if it does, the window is snapping
    # to the wrong lobe near the self-crossing.
    if np.any(np.abs(xte_comp) > error_comp + 1e-9):
        print("WARNING: |XTE| exceeds point-to-point error at some index "
              "(window may be snapping to the wrong lobe).")

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
    ax1.plot(time_list, xte_comp, color='tab:green',
             label='Position cross-track error (compensated)')
    ax1.axhline(y=0.0, color='k', linestyle='--', linewidth=0.8)
    ax1.set_ylabel('Position XTE (m)')
    ax1.set_title('Cross-Track Error: Reference vs Compensated Trajectory')
    ax1.legend()
    ax1.grid()
    ax2.plot(time_list, xte_theta, color='tab:red',
             label='Orientation cross-track error (compensated)')
    ax2.axhline(y=0.0, color='k', linestyle='--', linewidth=0.8)
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Heading XTE (rad)')
    ax2.legend()
    ax2.grid()
    plt.tight_layout()
    plt.show()
