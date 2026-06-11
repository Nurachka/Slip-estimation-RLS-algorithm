import sys
import os
sys.path.append("..")
from mathematical_simulator_class.robot import Robot
from mathematical_simulator_class.file_reader import Analysis
from mathematical_simulator_class.feedforward import Feedforward
from mathematical_simulator_class.recursive_least_square import RecursiveLeastSquares
import matplotlib.pyplot as plt
import numpy as np

# --- Parameters ---
SLIP_OFFSET  = 0.045         # center = (0.08 + 0.01) / 2
SLIP_AMP     = 0.035         # half-amplitude = (0.08 - 0.01) / 2
SLIP_FREQ_HZ = 0.05          # Hz — one full cycle every 400 timesteps at dt=0.05 s
DT           = 0.05

INITIAL_X     = 1.0
INITIAL_Y     = 0.0
INITIAL_THETA = 1.5786512774347865

LAMBDA_VALUES = [1.0, 0.99, 0.97, 0.95, 0.80]
USE_NOISE     = True        # set True to add orientation noise to the RLS estimator

# --- Load trajectory ---
file_reader    = Analysis()
current_dir    = os.path.dirname(os.path.abspath(__file__))
trajectory_dir = os.path.join(current_dir, '..', 'trajectories')
file_path      = os.path.join(trajectory_dir, 'lemniscate_trajectory.csv')
feedforward    = Feedforward(file_reader.read_csv(file_path))


def slip_fn(k):
    return SLIP_OFFSET + SLIP_AMP * np.sin(2 * np.pi * SLIP_FREQ_HZ * k * DT)


def run_simulation(lam=None, use_compensation=True, use_noise=False):
    """
    lam=None             -> uncompensated baseline (true slip applied, no RLS)
    lam=float            -> RLS with fixed forgetting factor
    use_noise=True       -> Gaussian orientation noise (NOISE_STD_ORIENTATION) added to
                           theta before feeding the RLS estimator; x, y stay clean.
    """
    robot = Robot(initial_x=INITIAL_X, initial_y=INITIAL_Y, initial_theta=INITIAL_THETA)

    if use_compensation:
        estimator = RecursiveLeastSquares(
            s0=np.array([0.0]), P0=10 * np.eye(1), R=0.00436 * np.eye(1, 1)
        )

    theta_previous = INITIAL_THETA
    states         = []
    slip_estimates = []
    covs           = []

    for k in range(len(feedforward.df)):
        robot.slip = slip_fn(k)
        vel_right, vel_left = feedforward.vel_at_timestep(k)

        if use_compensation:
            s_hat = np.clip(float(estimator.estimates[-1][0].item()), -0.5, 0.5)
            vel_right_comp = vel_right / (1 - s_hat)
            vel_left_comp  = vel_left  / (1 - s_hat)
        else:
            vel_right_comp = vel_right
            vel_left_comp  = vel_left

        x, y, theta = robot.forward_kinematics(vel_right_comp, vel_left_comp)
        if use_noise:
            _, _, theta_for_rls = robot.add_noise()
        else:
            theta_for_rls = theta

        if use_compensation:
            estimator.predict_sim_with_forgetting_factor(
                theta_for_rls, theta_previous, vel_right_comp, vel_left_comp, DT, lam=lam
            )
            slip_estimates.append(float(estimator.estimates[-1][0].item()))
            covs.append(float(estimator.estimationErrorCovarianceMatrices[-1][0].item()))

        theta_previous = theta_for_rls
        states.append(np.array([x, y, theta]))

    return np.array(states), slip_estimates, covs


# --- Run scenarios ---
states_actual, _, _ = run_simulation(use_compensation=False, use_noise=USE_NOISE)

rls_results = {}
for lam in LAMBDA_VALUES:
    states, slips, covs = run_simulation(lam=lam, use_noise=USE_NOISE)
    rls_results[lam] = (states, slips, covs)

# --- Shared data ---
x_target_list = feedforward.df['x'].tolist()
y_target_list = feedforward.df['y'].tolist()
N             = len(feedforward.df)
true_slip     = [slip_fn(k) for k in range(N)]

colors     = ['blue', 'orange', 'green', 'red', 'purple']
suffix     = ' (noisy)' if USE_NOISE else ''
labels_rls = [f'RLS λ={lam}{suffix}' for lam in LAMBDA_VALUES]


def aligned_error(states):
    aligned = np.vstack([np.array([INITIAL_X, INITIAL_Y]), states[:-1, :2]])
    reference_xy = np.column_stack([x_target_list, y_target_list])
    return np.linalg.norm(aligned - reference_xy, axis=1)


# --- Figure 1: Trajectory ---
plt.figure()
plt.plot(x_target_list, y_target_list, 'r--', label='Target')
plt.plot(states_actual[:, 0], states_actual[:, 1], color='gray', label='No compensation')
for (lam, (states, slips, covs)), color, label in zip(rls_results.items(), colors, labels_rls):
    plt.plot(states[:, 0], states[:, 1], color=color, label=label)
plt.xlabel('X Position (m)')
plt.ylabel('Y Position (m)')
plt.title('Trajectory Comparison — Sinusoidal Slip')
plt.legend()
plt.gca().set_aspect('equal', adjustable='box')
plt.grid()
plt.show()


# --- Figure 2: Slip estimates vs true slip ---
plt.figure(figsize=(12, 5))
plt.plot(true_slip, 'k--', linewidth=1.5, label='True slip')
for (lam, (states, slips, covs)), color, label in zip(rls_results.items(), colors, labels_rls):
    plt.plot(slips, color=color, label=label)
plt.xlabel('Time Step')
plt.ylabel('Slip')
plt.title('Slip Estimation — Fixed λ Comparison')
plt.legend()
plt.grid()
plt.tight_layout()
plt.show()


# --- Figure 3: Slip estimation error (estimate − true) ---
plt.figure(figsize=(12, 5))
for (lam, (states, slips, covs)), color, label in zip(rls_results.items(), colors, labels_rls):
    error = np.array(slips) - np.array(true_slip)
    plt.plot(error, color=color, label=label)
plt.axhline(0, color='k', linestyle='--', linewidth=0.8)
plt.xlabel('Time Step')
plt.ylabel('Estimation Error')
plt.title('Slip Estimation Error (estimate − true)')
plt.legend()
plt.grid()
plt.tight_layout()
plt.show()


# --- Figure 4: Trajectory tracking error ---
plt.figure(figsize=(12, 5))
plt.plot(aligned_error(states_actual), color='gray', label='No compensation')
for (lam, (states, slips, covs)), color, label in zip(rls_results.items(), colors, labels_rls):
    plt.plot(aligned_error(states), color=color, label=label)
plt.xlabel('Time Step')
plt.ylabel('Tracking Error (m)')
plt.title('Trajectory Tracking Error — Sinusoidal Slip')
plt.legend()
plt.grid()
plt.tight_layout()
plt.show()


# --- Figure 5: Estimation error covariance ---
plt.figure(figsize=(12, 5))
for (lam, (states, slips, covs)), color, label in zip(rls_results.items(), colors, labels_rls):
    plt.plot(covs, color=color, label=label)
plt.xlabel('Time Step')
plt.ylabel('Estimation Error Covariance')
plt.title('RLS Estimation Error Covariance — Sinusoidal Slip')
plt.legend()
plt.grid()
plt.tight_layout()
plt.show()


# --- Metrics ---
print("\n--- Slip Estimation RMS Error ---")
print(f"  {'Scenario':<26s}  {'RMS':>8s}")
for (lam, (states, slips, covs)), label in zip(rls_results.items(), labels_rls):
    rms = np.sqrt(np.mean((np.array(slips) - np.array(true_slip)) ** 2))
    print(f"  {label:<26s}: {rms:.5f}")

print("\n--- Trajectory Tracking Error ---")
err_actual = aligned_error(states_actual)
print(f"  {'No compensation':<26s}: min {err_actual.min():.3f} m, max {err_actual.max():.3f} m, mean {err_actual.mean():.3f} m")
for (lam, (states, slips, covs)), label in zip(rls_results.items(), labels_rls):
    err = aligned_error(states)
    print(f"  {label:<26s}: min {err.min():.3f} m, max {err.max():.3f} m, mean {err.mean():.3f} m")
