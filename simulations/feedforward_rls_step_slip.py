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
LAMBDA_HIGH  = 0.96
LAMBDA_LOW   = 0.95
WINDOW_START = 150
WINDOW_END   = 250
WARMUP       = 50

INITIAL_X     = 1.0
INITIAL_Y     = 0.0
INITIAL_THETA = 1.5786512774347865

# --- Load trajectory ---
file_reader    = Analysis()
current_dir    = os.path.dirname(os.path.abspath(__file__))
trajectory_dir = os.path.join(current_dir, '..', 'trajectories')
file_path      = os.path.join(trajectory_dir, 'lemniscate_trajectory.csv')
feedforward    = Feedforward(file_reader.read_csv(file_path))


def run_simulation(slip_fn, lam_fn=None, use_compensation=True):
    """
    slip_fn(k)  -> true slip at timestep k
    lam_fn(k)   -> forgetting factor lambda at timestep k, or None for standard RLS
    use_compensation -> False runs the plain uncompensated baseline
    Returns: (states, slip_estimates, covs, lambda_list)
    """
    robot = Robot(initial_x=INITIAL_X, initial_y=INITIAL_Y, initial_theta=INITIAL_THETA)

    if use_compensation:
        estimator = RecursiveLeastSquares(s0=np.array([0.0]), P0=10*np.eye(1, 1), R=0.00436*np.eye(1, 1))

    theta_previous = INITIAL_THETA
    states         = []
    slip_estimates = []
    covs           = []
    lambda_list    = []

    for timestep in range(len(feedforward.df)):
        robot.slip = slip_fn(timestep)
        vel_right, vel_left = feedforward.vel_at_timestep(timestep)

        if use_compensation:
            s_hat = np.clip(float(estimator.estimates[-1][0]), -0.5, 0.5)
            if timestep > WARMUP and s_hat > 0.0:
                vel_right_comp = vel_right / (1 - s_hat)
                vel_left_comp  = vel_left  / (1 - s_hat)
            else:
                vel_right_comp = vel_right
                vel_left_comp  = vel_left
        else:
            vel_right_comp = vel_right
            vel_left_comp  = vel_left

        x, y, theta        = robot.forward_kinematics(vel_right_comp, vel_left_comp)
        _, _, theta_noised = robot.add_noise()

        if use_compensation:
            lam = lam_fn(timestep) if lam_fn is not None else None
            if lam is not None:
                estimator.predict_sim_with_forgetting_factor(
                    theta_noised, theta_previous, vel_right_comp, vel_left_comp, 0.05, lam=lam
                )
            else:
                estimator.predict_sim(theta_noised, theta_previous, vel_right_comp, vel_left_comp, 0.05)
            slip_estimates.append(float(estimator.estimates[-1][0]))
            covs.append(estimator.estimationErrorCovarianceMatrices[timestep][0])
            lambda_list.append(lam if lam is not None else 1.0)

        theta_previous = theta_noised
        states.append(np.array([x, y, theta]))

    return np.array(states), slip_estimates, covs, lambda_list


# --- Slip and lambda profiles ---
def slip_fn(k):
    return 0.2 if k >= 200 else 0.0

def lam_var_fn(k):
    return LAMBDA_LOW if WINDOW_START <= k <= WINDOW_END else LAMBDA_HIGH


# --- Run scenarios ---
states_actual, _,           _,          _           = run_simulation(slip_fn, use_compensation=False)
states_rls,    slips_rls,   covs_rls,   _           = run_simulation(slip_fn)
states_const,  slips_const, covs_const, _           = run_simulation(slip_fn, lam_fn=lambda k: LAMBDA_HIGH)
states_var,    slips_var,   covs_var,   lambda_list = run_simulation(slip_fn, lam_fn=lam_var_fn)


# --- Shared plot data ---
x_target_list = feedforward.df['x'].tolist()
y_target_list = feedforward.df['y'].tolist()
true_slip     = [slip_fn(k) for k in range(len(feedforward.df))]

labels     = ['Actual (no comp)', 'RLS (no forgetting)', f'RLS λ={LAMBDA_HIGH} (const)', f'RLS λ={LAMBDA_LOW}/{LAMBDA_HIGH} (var)']
colors     = ['gray', 'blue', 'orange', 'green']
all_states = [states_actual, states_rls, states_const, states_var]


# --- Figure 1: Trajectory ---
plt.figure()
plt.plot(x_target_list, y_target_list, 'r--', label='Target')
for states, label, color in zip(all_states, labels, colors):
    plt.plot(states[:, 0], states[:, 1], color=color, label=label)
plt.xlabel('X Position (m)')
plt.ylabel('Y Position (m)')
plt.title('Trajectory Comparison')
plt.legend()
plt.gca().set_aspect('equal', adjustable='box')
plt.grid()
plt.show()


# --- Figure 2: Slip estimates ---
fig, ax1 = plt.subplots()
ax1.plot(true_slip, 'k--', label='True slip')
for slips, label, color in zip([slips_rls, slips_const, slips_var], labels[1:], colors[1:]):
    ax1.plot(slips, color=color, label=label)
ax1.set_xlabel('Time Step')
ax1.set_ylabel('Slip Value')
ax1.set_title('Estimated Slip and Forgetting Factor Over Time')
ax2 = ax1.twinx()
ax2.plot(lambda_list, color='tab:green', linestyle=':', linewidth=1.5, label='λ(t)')
ax2.set_ylabel('λ')
ax2.set_ylim(0.93, 1.01)
lines1, labels1 = ax1.get_legend_handles_labels()
lines2, labels2 = ax2.get_legend_handles_labels()
ax1.legend(lines1 + lines2, labels1 + labels2)
ax1.grid()
plt.show()


# --- Figure 3: Tracking error ---
INITIAL      = np.array([INITIAL_X, INITIAL_Y, INITIAL_THETA])
reference_xy = np.column_stack([x_target_list, y_target_list])

def aligned_error(states):
    aligned = np.vstack([INITIAL, states[:-1]])
    return np.linalg.norm(aligned[:, :2] - reference_xy, axis=1)

errors = [aligned_error(s) for s in all_states]

plt.figure(figsize=(10, 4))
for err, label, color in zip(errors, labels, colors):
    plt.plot(err, color=color, label=label)
plt.xlabel('Time Step')
plt.ylabel('Tracking Error (m)')
plt.title('Tracking Error Over Time')
plt.legend()
plt.grid()
plt.tight_layout()
plt.show()

for err, label in zip(errors, labels):
    print(f"{label:40s}: min {err.min():.3f} m, max {err.max():.3f} m, mean {err.mean():.3f} m")


# --- Figure 4: Estimation error covariance ---
plt.figure(figsize=(10, 4))
for covs, label, color in zip([covs_rls, covs_const, covs_var], labels[1:], colors[1:]):
    plt.plot(covs, color=color, label=label)
plt.xlabel('Time Step')
plt.ylabel('Estimation Error Covariance')
plt.title('RLS Estimation Error Covariance Over Time')
plt.legend()
plt.grid()
plt.tight_layout()
plt.show()
