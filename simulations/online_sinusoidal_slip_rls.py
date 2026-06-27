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

###Online RLS slip compensation with a sinusoidally varying true slip###
# Same online feedforward + RLS structure as online_fixed_slip_rls.py, but the true wheel slip
# oscillates 0.1-0.2 at 0.05 Hz (one full cycle every 400 timesteps at dt=0.05 s) instead of
# being fixed. To follow a moving slip the RLS uses a forgetting factor (lambda = 0.97) so it
# down-weights old measurements. Compares a compensated run (feedforward + RLS) against an
# uncompensated baseline, and reports how well the estimate tracks the known sinusoid.

# at t=0 the robot is at (1, 0) with theta ≈ π/2 so the lemniscate is horizontal
INITIAL_X     = 1.0
INITIAL_Y     = 0.0
INITIAL_THETA = 1.5786512774347865

# Sinusoidal slip profile — oscillates 0.1-0.2
SLIP_OFFSET  = 0.15          # center = (0.2 + 0.1) / 2
SLIP_AMP     = 0.05          # half-amplitude = (0.2 - 0.1) / 2
SLIP_FREQ_HZ = 0.05          # Hz — one full cycle every 400 timesteps at dt=0.05 s
DT           = TIMESTEP      # 0.05 s

# Adaptive forgetting factor, gated on excitation = |angular velocity ω| (the RLS regressor
# magnitude). Slip is only observable while the robot turns, so λ rides up toward LAMBDA_MAX
# (no forgetting) on straight segments and drops toward LAMBDA_MIN (forget/track) when turning.
LAMBDA_MIN  = 0.95           # most forgetting, when |ω| is large (turning)
LAMBDA_MAX  = 1.0            # no forgetting, when |ω| ≈ 0 (straight line)
OMEGA_SCALE = 0.25           # |ω| (rad/s) scale controlling how fast λ ramps from MAX → MIN
                             # (≈0.25 ≈ |ω|_max/4 here: near-straight stays ~1.0, typical turning ~0.97)

SEED         = 42            # fixed RNG seed so the sensor noise is identical every run
STEADY_STATE_START = 100


def slip_fn(k):
    return SLIP_OFFSET + SLIP_AMP * np.sin(2 * np.pi * SLIP_FREQ_HZ * k * DT)


def adaptive_lambda(omega):
    # Smooth, saturating ramp: λ = LAMBDA_MAX at ω=0, → LAMBDA_MIN as |ω| grows. Bounded in
    # [LAMBDA_MIN, LAMBDA_MAX] by construction, so the transition has no discontinuity.
    ramp = 1.0 - np.exp(-abs(omega) / OMEGA_SCALE)
    return LAMBDA_MAX - (LAMBDA_MAX - LAMBDA_MIN) * ramp


def run_simulation(feedforward):
    np.random.seed(SEED)
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
    omega_list           = []
    lambda_list          = []

    theta_previous_noised = INITIAL_THETA
    slip_previous = 0.0

    for timestep in range(len(feedforward.df)):
        # inject the time-varying true slip on both robots before they move
        robot.slip         = slip_fn(timestep)
        robot_no_comp.slip = slip_fn(timestep)

        vel_right, vel_left = feedforward.vel_at_timestep(timestep)
        vel_right_list.append(vel_right)
        vel_left_list.append(vel_left)

        slip_clamped  = max(min(slip_previous, 0.5), 0.0)
        vel_right_comp = vel_right / (1 - slip_clamped)
        vel_left_comp  = vel_left  / (1 - slip_clamped)
        vel_right_comp_list.append(vel_right_comp)
        vel_left_comp_list.append(vel_left_comp)

        x_a, y_a, theta_a = robot.forward_kinematics(vel_right_comp, vel_left_comp)
        x_noised, y_noised, theta_noised = robot.add_noise()
        x_nc, y_nc, theta_nc = robot_no_comp.forward_kinematics(vel_right, vel_left)
        no_comp_trajectory.append((x_nc, y_nc, theta_nc))

        # excitation gate uses the same commanded angular velocity the RLS regressor sees
        omega = (vel_right_comp - vel_left_comp) / WHEEL_BASE
        lam_k = adaptive_lambda(omega)
        omega_list.append(omega)
        lambda_list.append(lam_k)

        estimator.predict_sim_with_forgetting_factor(theta_noised, theta_previous_noised,
                                                     vel_right_comp, vel_left_comp, TIMESTEP, lam=lam_k)
        theta_previous_noised = theta_noised

        comp_trajectory.append((x_a, y_a, theta_a))
        slip_previous = estimator.estimates[-1][0]
        slip.append(slip_previous)

    return (robot_no_comp, comp_trajectory, no_comp_trajectory, slip,
            vel_right_list, vel_left_list, vel_right_comp_list, vel_left_comp_list,
            omega_list, lambda_list)


if __name__ == "__main__":
    file_reader    = Analysis()
    current_dir    = os.path.dirname(os.path.abspath(__file__))
    trajectory_dir = os.path.join(current_dir, '..', 'trajectories')
    file_path      = os.path.join(trajectory_dir, 'lemniscate_trajectory.csv')
    feedforward    = Feedforward(file_reader.read_csv(file_path))

    robot_no_comp, comp_trajectory, no_comp_trajectory, slip, \
        vel_right_list, vel_left_list, vel_right_comp_list, vel_left_comp_list, \
        omega_list, lambda_list = run_simulation(feedforward)

    x_target_list     = feedforward.df['x'].tolist()
    y_target_list     = feedforward.df['y'].tolist()
    theta_target_list = feedforward.df['theta'].tolist()

    true_slip = np.array([slip_fn(k) for k in range(len(feedforward.df))])

    print("Estimated slip values:", slip[-1])

    abs_omega = np.abs(omega_list)
    lam_arr   = np.array(lambda_list)
    print(f"|ω| (rad/s)  — mean: {abs_omega.mean():.4f}, max: {abs_omega.max():.4f}")
    print(f"Adaptive λ   — min: {lam_arr.min():.4f}, mean: {lam_arr.mean():.4f}, max: {lam_arr.max():.4f}")

    plt.plot(true_slip, 'k--', linewidth=1.5, label='True slip')
    plt.plot(slip, label='Estimated Slip (adaptive λ)')
    plt.xlabel('Time Step')
    plt.ylabel('Slip')
    plt.title('Estimated Slip Over Time — Sinusoidal Slip')
    plt.legend()
    plt.grid()
    plt.show()

    # Adaptive forgetting factor vs excitation |ω|
    fig, ax_lam = plt.subplots(figsize=(10, 4))
    ax_lam.plot(lambda_list, color='tab:blue', label='Adaptive λ')
    ax_lam.set_xlabel('Time Step')
    ax_lam.set_ylabel('λ', color='tab:blue')
    ax_lam.tick_params(axis='y', labelcolor='tab:blue')
    ax_lam.grid()
    ax_omega = ax_lam.twinx()
    ax_omega.plot(abs_omega, color='tab:red', alpha=0.6, label='|ω| (rad/s)')
    ax_omega.set_ylabel('|ω| (rad/s)', color='tab:red')
    ax_omega.tick_params(axis='y', labelcolor='tab:red')
    ax_lam.set_title('Adaptive Forgetting Factor vs Excitation |ω|')
    fig.tight_layout()
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

    no_comp_x_aligned     = [INITIAL_X]     + [x     for x, y, theta in no_comp_trajectory[:-1]]
    no_comp_y_aligned     = [INITIAL_Y]     + [y     for x, y, theta in no_comp_trajectory[:-1]]
    no_comp_theta_aligned = [INITIAL_THETA] + [theta for _, _, theta  in no_comp_trajectory[:-1]]
    error_no_comp = np.linalg.norm(
        np.column_stack((x_target_list, y_target_list)) -
        np.column_stack((no_comp_x_aligned, no_comp_y_aligned)), axis=1)
    heading_error_no_comp = np.abs(np.arctan2(
        np.sin(np.array(theta_target_list) - np.array(no_comp_theta_aligned)),
        np.cos(np.array(theta_target_list) - np.array(no_comp_theta_aligned))))

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

    print("--- Compensated (feedforward + RLS) ---")
    print(f"Position error  — mean: {error_comp.mean():.4f} m,  max: {error_comp.max():.4f} m")
    print(f"Heading error   — mean: {heading_error.mean():.4f} rad, max: {heading_error.max():.4f} rad")
    print(f"Position error  — RMSE: {np.sqrt(np.mean(error_comp**2)):.4f} m")
    print(f"Heading error   — RMSE: {np.sqrt(np.mean(heading_error**2)):.4f} rad")
    print(f"Position error  — std dev: {error_comp.std():.4f} m")
    print(f"Heading error   — std dev: {heading_error.std():.4f} rad")

    print("--- Feedforward only (no RLS compensation) ---")
    print(f"Position error  — mean: {error_no_comp.mean():.4f} m,  max: {error_no_comp.max():.4f} m")
    print(f"Heading error   — mean: {heading_error_no_comp.mean():.4f} rad, max: {heading_error_no_comp.max():.4f} rad")
    print(f"Position error  — RMSE: {np.sqrt(np.mean(error_no_comp**2)):.4f} m")
    print(f"Heading error   — RMSE: {np.sqrt(np.mean(heading_error_no_comp**2)):.4f} rad")
    print(f"Position error  — std dev: {error_no_comp.std():.4f} m")
    print(f"Heading error   — std dev: {heading_error_no_comp.std():.4f} rad")

    # Slip-estimation accuracy against the time-varying true slip
    slips_arr = np.array([float(s) for s in slip])
    rmse_full = np.sqrt(np.mean((slips_arr - true_slip) ** 2))
    mae_full  = np.mean(np.abs(slips_arr - true_slip))
    rmse_ss   = np.sqrt(np.mean((slips_arr[STEADY_STATE_START:] - true_slip[STEADY_STATE_START:]) ** 2))
    mae_ss    = np.mean(np.abs(slips_arr[STEADY_STATE_START:] - true_slip[STEADY_STATE_START:]))
    print("--- Slip estimation (vs true sinusoidal slip) ---")
    print(f"Full run            — RMSE: {rmse_full:.4f}, MAE: {mae_full:.4f}")
    print(f"Steps {STEADY_STATE_START}→end        — RMSE: {rmse_ss:.4f}, MAE: {mae_ss:.4f}")

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
