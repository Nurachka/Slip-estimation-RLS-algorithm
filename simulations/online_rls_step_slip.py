import sys
import os
sys.path.append("..")
from mathematical_simulator_class.config import NOISE_STD_POSITION, NOISE_STD_ORIENTATION
from mathematical_simulator_class.robot import Robot
from mathematical_simulator_class.file_reader import Analysis
from mathematical_simulator_class.feedforward import Feedforward
from mathematical_simulator_class.recursive_least_square import RecursiveLeastSquares
from mathematical_simulator_class.compensator import Compensator
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

###Online RLS slip estimation with constant forgetting factor###

# --- Parameters ---
LAMBDA = 0.96        # constant forgetting factor
SLIP_STEP_TIME = 200  # timestep at which the true slip jumps
SLIP_BEFORE = 0.0     # true slip before the step
SLIP_AFTER = 0.2      # true slip after the step
SEED = 42             # RNG seed so measurement noise matches the MPC tests

file_reader = Analysis()
robot = Robot(initial_x=1.0, initial_y=0.0, initial_theta=1.5786512774347865)
robot_no_comp = Robot(initial_x=1.0, initial_y=0.0, initial_theta=1.5786512774347865)
current_dir = os.path.dirname(os.path.abspath(__file__))
trajectory_dir = os.path.join(current_dir, '..', 'trajectories')
file_path = os.path.join(trajectory_dir, 'lemniscate_trajectory.csv')

np.random.seed(SEED)  # seed RNG so add_noise() draws match test_linear_mpc_lemniscate.py

feedforward = Feedforward(file_reader.read_csv(file_path))
estimator = RecursiveLeastSquares(s0=np.array([0.0]), P0=50*np.eye(1), R=2*NOISE_STD_ORIENTATION**2*np.eye(1))
slip = []
x_a, y_a, theta_a = 1.0, 0.0, 1.5786512774347865
comp_trajectory = []
vel_right_list = []
vel_left_list = []
vel_right_comp_list = []
vel_left_comp_list = []

theta_previous_noised = 1.5786512774347865
slip_previous = 0.0
true_slip_list = []

for timestep in range(len(feedforward.df)):

    # true slip is a step function: SLIP_BEFORE, then jumps to SLIP_AFTER
    robot.slip = SLIP_BEFORE if timestep < SLIP_STEP_TIME else SLIP_AFTER
    robot_no_comp.slip = robot.slip
    true_slip_list.append(robot.slip)

    vel_right, vel_left = feedforward.vel_at_timestep(timestep)
    vel_right_list.append(vel_right)
    vel_left_list.append(vel_left)

    slip_clamped = max(min(slip_previous, 0.2), 0.0)
    vel_right_comp = vel_right / (1 - slip_clamped)
    vel_left_comp = vel_left / (1 - slip_clamped)
    vel_right_comp_list.append(vel_right_comp)
    vel_left_comp_list.append(vel_left_comp)

    x_a, y_a, theta_a = robot.forward_kinematics(vel_right_comp, vel_left_comp)
    x_noised, y_noised, theta_noised = robot.add_noise()

    # actual trajectory with slip, driven by nominal (uncompensated) velocities
    robot_no_comp.forward_kinematics(vel_right, vel_left)

    estimator.predict_sim_with_forgetting_factor(
        theta_noised, theta_previous_noised, vel_right_comp, vel_left_comp, 0.05, lam=LAMBDA
    )
    theta_previous_noised = theta_noised

    comp_trajectory.append((x_a, y_a, theta_a))

    slip_previous = estimator.estimates[-1][0]
    slip.append(slip_previous)


x_target_list = feedforward.df['x'].tolist()
y_target_list = feedforward.df['y'].tolist()
time_list = feedforward.df['time'].tolist()  # seconds

plt.plot([x for x, y, theta in comp_trajectory], [y for x, y, theta in comp_trajectory], label='Compensated Robot Path', linestyle='--')
plt.plot(x_target_list, y_target_list, label='Target Path', linestyle=':')
plt.plot(robot.x_list, robot.y_list, label='Robot Path without compensation', linestyle=':')
plt.xlabel('X Position (m)')
plt.ylabel('Y Position (m)')
plt.title('Compensated Robot Path')
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

print("Estimated slip values:", slip[-1])

# Slip over time
plt.plot(time_list, slip, label='Estimated Slip', color='blue')
plt.plot(time_list, true_slip_list, label='True Slip', color='red', linestyle='--')
plt.xlabel('Time (s)')
plt.ylabel('Slip')
plt.title(f'Estimated Slip Over Time (λ = {LAMBDA})')
plt.legend()
plt.grid()
plt.show()

# Estimation error covariance P_k over time (drop the initial P0 so it aligns with time_list)
P_list = [float(np.ravel(P)[0]) for P in estimator.estimationErrorCovarianceMatrices[1:]]
plt.figure(figsize=(7, 4))
plt.plot(time_list, P_list, label='Estimation error covariance P', color='green')
plt.axvline(x=time_list[SLIP_STEP_TIME], color='tab:gray', linestyle=':', label='Slip step')
plt.xlabel('Time (s)')
plt.ylabel('P (covariance)')
plt.title(f'Estimation Error Covariance Over Time (λ = {LAMBDA})')
plt.legend()
plt.grid()
plt.tight_layout()
plt.show()

# Kalman gain K_k over time (one gain matrix per timestep)
K_list = [float(np.ravel(K)[0]) for K in estimator.gainMatrices]
plt.figure(figsize=(7, 4))
plt.plot(time_list, K_list, label='Kalman gain K', color='purple')
plt.axvline(x=time_list[SLIP_STEP_TIME], color='tab:gray', linestyle=':', label='Slip step')
plt.xlabel('Time (s)')
plt.ylabel('K (gain)')
plt.title(f'Kalman Gain Over Time (λ = {LAMBDA})')
plt.legend()
plt.grid()
plt.tight_layout()
plt.show()

# Time after the slip step at which the estimate re-converges to the true value
THRESHOLD = 0.01
slip_arr = np.array(slip)
# consider only the post-step region where true slip = SLIP_AFTER
within_after = np.abs(slip_arr[SLIP_STEP_TIME:] - SLIP_AFTER) <= THRESHOLD
if not within_after.any():
    print(f"Estimated slip never converged within {THRESHOLD} of true slip {SLIP_AFTER} after the step")
else:
    # first step (relative to the step) from which the estimate stays within threshold
    rel_idx = np.where(~within_after)[0][-1] + 1 if (~within_after).any() else 0
    conv_idx = SLIP_STEP_TIME + rel_idx
    if conv_idx < len(slip_arr):
        print(f"Estimated slip converged within {THRESHOLD} of true slip {SLIP_AFTER} "
              f"at t = {time_list[conv_idx]:.2f} s (step {conv_idx}), "
              f"{time_list[conv_idx] - time_list[SLIP_STEP_TIME]:.2f} s after the step")
    else:
        print(f"Estimated slip never settled within {THRESHOLD} of true slip {SLIP_AFTER} after the step")

#plot the target trajectory from calculated feedforward velocities
x_ref_list = x_target_list
y_ref_list = y_target_list
# reference trajectory with Gaussian position measurement noise
x_ref_noised = np.array(x_ref_list) + np.random.normal(0, NOISE_STD_POSITION, len(x_ref_list))
y_ref_noised = np.array(y_ref_list) + np.random.normal(0, NOISE_STD_POSITION, len(y_ref_list))
plt.plot(x_ref_list, y_ref_list,  color='blue')
plt.plot(x_ref_noised, y_ref_noised, 
         color='blue', alpha=0.4, linewidth=0.8)
plt.plot([x for x, y, theta in comp_trajectory], [y for x, y, theta in comp_trajectory], color='purple' )
plt.xlabel('X Position (m)')
plt.ylabel('Y Position (m)')
plt.legend()
plt.axis('equal')
plt.grid()
plt.show()

comp_x_aligned = [x_a] + [x for x, y, theta in comp_trajectory[:-1]]
comp_y_aligned = [y_a] + [y for x, y, theta in comp_trajectory[:-1]]
error_comp = np.linalg.norm(np.column_stack((x_target_list, y_target_list)) - np.column_stack((comp_x_aligned, comp_y_aligned)), axis=1)
error_comp = error_comp[1:]  # drop the first value (initial transient)
plt.figure(figsize=(7, 4))
plt.plot(time_list[1:], error_comp)
plt.xlabel('Time (s)')
plt.ylabel('Error in position (m)')
plt.legend()
plt.grid()
plt.show()

# tracking orientation error over time
theta_target_list = feedforward.df['theta'].tolist()
comp_theta_aligned = [theta_a] + [theta for x, y, theta in comp_trajectory[:-1]]
heading_error = np.abs(np.arctan2(
    np.sin(np.array(theta_target_list) - np.array(comp_theta_aligned)),
    np.cos(np.array(theta_target_list) - np.array(comp_theta_aligned))))
heading_error = heading_error[1:]  # drop the first value (initial transient)
plt.figure(figsize=(7, 4))
plt.plot(time_list[1:], heading_error)
plt.xlabel('Time (s)')
plt.ylabel('Error in heading (rad)')
plt.legend()
plt.grid()
plt.show()


# print(f"Position error between compensated trajectory and target trajectory: min {error_comp.min():.3f} m, max {error_comp.max():.3f} m, mean {error_comp.mean():.3f} m")
# print(f"Orientation tracking error between compensated trajectory and target trajectory: min {heading_error.min():.3f} rad, max {heading_error.max():.3f} rad, mean {heading_error.mean():.3f} rad")

# ----- Tracking error: noisy MEASURED actual pose (RLS-compensated) vs reference -----
# Uses the Gaussian-noised measurements collected in the compensated robot
# (robot.*_list_noised), aligned/trimmed the same way as error_comp above.
meas_x_aligned     = [x_noised]     + robot.x_list_noised[:-1]
meas_y_aligned     = [y_noised]     + robot.y_list_noised[:-1]
meas_theta_aligned = [theta_noised] + robot.theta_list_noised[:-1]
error_meas = np.linalg.norm(
    np.column_stack((x_target_list, y_target_list)) -
    np.column_stack((meas_x_aligned, meas_y_aligned)), axis=1)
error_meas = error_meas[1:]  # drop the first value (initial transient)
heading_error_meas = np.abs(np.arctan2(
    np.sin(np.array(theta_target_list) - np.array(meas_theta_aligned)),
    np.cos(np.array(theta_target_list) - np.array(meas_theta_aligned))))
heading_error_meas = heading_error_meas[1:]  # drop the first value (initial transient)

# Measured position tracking error
plt.figure(figsize=(7, 4))
plt.plot(time_list[1:], error_meas, color='blue')
plt.axvline(x=time_list[SLIP_STEP_TIME], color='tab:gray', linestyle=':', label='Slip step')
plt.xlabel('Time (s)')
plt.ylabel('Error in position (m)')
plt.legend()
plt.grid()
plt.tight_layout()
plt.show()

# Measured heading tracking error
plt.figure(figsize=(7, 4))
plt.plot(time_list[1:], heading_error_meas, color='blue')
plt.axvline(x=time_list[SLIP_STEP_TIME], color='tab:gray', linestyle=':', label='Slip step')
plt.xlabel('Time (s)')
plt.ylabel('Error in heading (rad)')
plt.legend()
plt.grid()
plt.tight_layout()
plt.show()

print("--- Measured (noisy) actual pose with RLS vs reference ---")
print(f"Position error - mean: {error_meas.mean():.4f} m, max: {error_meas.max():.4f} m, RMSE: {np.sqrt(np.mean(error_meas**2)):.4f} m")
print(f"Heading error  - mean: {heading_error_meas.mean():.4f} rad, max: {heading_error_meas.max():.4f} rad, RMSE: {np.sqrt(np.mean(heading_error_meas**2)):.4f} rad")

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

#plot the angular velocity of the robot over time
angular_velocity = [(vel_right - vel_left) / 0.1 for vel_right, vel_left in zip(vel_right_comp_list, vel_left_comp_list)]
plt.plot(time_list, angular_velocity, label='Angular Velocity (compensated)')
plt.xlabel('Time (s)')
plt.ylabel('Angular Velocity (rad/s)')
plt.title('Angular Velocity Over Time')
plt.legend()
plt.grid()
plt.show()


# ----- Cross-track error (Method 2: perpendicular distance to reference polyline) -----
theta_target_list  = feedforward.df['theta'].tolist()
comp_theta_aligned = [theta_a] + [theta for x, y, theta in comp_trajectory[:-1]]


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
xte_comp  = np.empty(len(comp_x_aligned))
xte_theta = np.empty(len(comp_x_aligned))  # orientation cross-track error (rad)
for i, (px, py, pth) in enumerate(zip(comp_x_aligned, comp_y_aligned, comp_theta_aligned)):
    signed, j, t = nearest_ref_match(px, py, i, x_target_list, y_target_list, XTE_WINDOW)
    xte_comp[i] = signed
    # reference heading at the nearest point: interpolate theta[j]..theta[j+1]
    # by the projection fraction t, wrapping the increment to stay on the circle.
    theta_ref_near = theta_target_list[j] + t * wrap(theta_target_list[j + 1] - theta_target_list[j])
    xte_theta[i] = wrap(pth - theta_ref_near)  # signed: + robot rotated left of path

# drop the first value (initial transient), matching error_comp above
xte_comp  = xte_comp[1:]
xte_theta = xte_theta[1:]

print("--- Cross-track error (reference vs compensated) ---")
print(f"XTE position - mean |e|: {np.abs(xte_comp).mean():.4f} m")
print(f"XTE position - max  |e|: {np.abs(xte_comp).max():.4f} m")
print(f"XTE position - RMSE   : {np.sqrt(np.mean(xte_comp**2)):.4f} m")
print(f"XTE heading  - mean |e|: {np.abs(xte_theta).mean():.4f} rad")
print(f"XTE heading  - max  |e|: {np.abs(xte_theta).max():.4f} rad")
print(f"XTE heading  - RMSE   : {np.sqrt(np.mean(xte_theta**2)):.4f} rad")

# sanity check: perpendicular (cross-track) error must not exceed the
# time-matched point-to-point distance; if it does, the window is snapping
# to the wrong lobe near the self-crossing.
if np.any(np.abs(xte_comp) > error_comp + 1e-9):
    print("WARNING: |XTE| exceeds point-to-point error at some index "
          "(window may be snapping to the wrong lobe).")

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
ax1.plot(time_list[1:], xte_comp, color='tab:green',
         label='Position cross-track error (compensated)')
ax1.axhline(y=0.0, color='k', linestyle='--', linewidth=0.8)
ax1.axvline(x=time_list[SLIP_STEP_TIME], color='tab:gray', linestyle=':',
            label='Slip step')
ax1.set_ylabel('Position XTE (m)')
ax1.set_title('Cross-Track Error: Reference vs Compensated Trajectory')
ax1.legend()
ax1.grid()
ax2.plot(time_list[1:], xte_theta, color='tab:red',
         label='Orientation cross-track error (compensated)')
ax2.axhline(y=0.0, color='k', linestyle='--', linewidth=0.8)
ax2.axvline(x=time_list[SLIP_STEP_TIME], color='tab:gray', linestyle=':',
            label='Slip step')
ax2.set_xlabel('Time (s)')
ax2.set_ylabel('Heading XTE (rad)')
ax2.legend()
ax2.grid()
plt.tight_layout()
plt.show()
