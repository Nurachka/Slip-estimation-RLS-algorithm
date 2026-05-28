import sys
import os
sys.path.append("..")
from mathematical_simulator_class.robot import Robot
from mathematical_simulator_class.file_reader import Analysis
from mathematical_simulator_class.feedforward import Feedforward
from mathematical_simulator_class.recursive_least_square import RecursiveLeastSquares
from mathematical_simulator_class.linear_mpc import LinearMPC
from mathematical_simulator_class.config import NOISE_STD_POSITION, NOISE_STD_ORIENTATION
import matplotlib.pyplot as plt
import numpy as np

# --- Constants ---
INITIAL_X     = 1.0
INITIAL_Y     = 0.0
INITIAL_THETA = 1.5786512774347865
TRUE_SLIP     = 0.2
WARMUP_STEPS  = 50
SLIP_CLIP     = 0.25
DELTA_T       = 0.05

# --- Load trajectory ---
file_reader    = Analysis()
current_dir    = os.path.dirname(os.path.abspath(__file__))
trajectory_dir = os.path.join(current_dir, '..', 'trajectories')
file_path      = os.path.join(trajectory_dir, 'lemniscate_trajectory.csv')
feedforward    = Feedforward(file_reader.read_csv(file_path))

n_steps = len(feedforward.df)

reference_states = []
for k in range(n_steps):
    x     = feedforward.x_y_at_timestep(k)[0]
    y     = feedforward.x_y_at_timestep(k)[1]
    theta = feedforward.theta_at_timestep(k)
    reference_states.append((x, y, theta))
reference_states = np.array(reference_states)

# --- Initialise components ---
actual_robot  = Robot(initial_x=INITIAL_X, initial_y=INITIAL_Y, initial_theta=INITIAL_THETA)
rls           = RecursiveLeastSquares(s0=np.array([0.0]), P0=10.0 * np.eye(1), R=0.0004 * np.eye(1))
mpc_ns        = LinearMPC(dt=DELTA_T, wheel_base=0.5, N_horizon=10, s=0.0)   # s fixed at 0
mpc_rls       = LinearMPC(dt=DELTA_T, wheel_base=0.5, N_horizon=10, s=0.0)   # s updated by RLS

theta_prev       = INITIAL_THETA
actual_states       = []
actual_states_clean = []
comp_vel_ns      = []   # corrected velocities — MPC with no slip model
comp_vel_rls     = []   # corrected velocities — MPC with RLS slip estimate
slip_estimates   = []

# --- Shared online loop ---
# Both controllers observe the same noisy state from actual_robot each step.
# mpc_ns always uses s=0; mpc_rls uses s_hat from RLS.
for k in range(n_steps):
    actual_robot.slip = TRUE_SLIP
    vel_right, vel_left = feedforward.vel_at_timestep(k)

    x, y, theta = actual_robot.forward_kinematics(vel_right, vel_left)
    noise_x     = np.random.normal(0, NOISE_STD_POSITION)
    noise_y     = np.random.normal(0, NOISE_STD_POSITION)
    noise_theta = np.random.normal(0, NOISE_STD_ORIENTATION)
    x_n     = x     + noise_x
    y_n     = y     + noise_y
    theta_n = theta + noise_theta
    actual_states.append((x_n, y_n, theta_n))
    actual_states_clean.append((x, y, theta))

    # RLS update
    rls.predict_sim(theta_n, theta_prev, vel_right, vel_left, delta_t=DELTA_T)
    theta_prev = theta_n

    s_hat = float(np.clip(rls.estimates[-1][0], -SLIP_CLIP, SLIP_CLIP))
    # if k < WARMUP_STEPS:
    #     s_hat = 0.0
    slip_estimates.append(s_hat)

    current_state = np.array([x_n, y_n, theta_n])
    ref_state     = reference_states[k]

    # Build A/B lists (same reference linearisation for both controllers)
    A_list, B_list_ns, B_list_rls = [], [], []
    for i in range(mpc_ns.N):
        future_idx              = min(k + i, n_steps - 1)
        theta_ref_i             = feedforward.theta_at_timestep(future_idx)
        vel_right_i, vel_left_i = feedforward.vel_at_timestep(future_idx)

        mpc_ns.s  = 0.0
        A_i, B_i_ns  = mpc_ns.define_AB_matrices(theta_ref_i, vel_right_i, vel_left_i)

        mpc_rls.s = s_hat
        _, B_i_rls = mpc_rls.define_AB_matrices(theta_ref_i, vel_right_i, vel_left_i)

        A_list.append(A_i)
        B_list_ns.append(B_i_ns)
        B_list_rls.append(B_i_rls)

    # MPC no-slip solve
    mpc_ns.s = 0.0
    error_ns = mpc_ns.compute_error_state(current_state, ref_state)
    dvr_ns, dvl_ns = mpc_ns.solve(error_ns, A_list, B_list_ns)
    comp_vel_ns.append((vel_right + dvr_ns, vel_left + dvl_ns))

    # MPC + RLS solve
    mpc_rls.s = s_hat
    error_rls = mpc_rls.compute_error_state(current_state, ref_state)
    dvr_rls, dvl_rls = mpc_rls.solve(error_rls, A_list, B_list_rls)
    comp_vel_rls.append((vel_right + dvr_rls, vel_left + dvl_rls))

actual_states       = np.array(actual_states)
actual_states_clean = np.array(actual_states_clean)
comp_vel_ns    = np.array(comp_vel_ns)
comp_vel_rls   = np.array(comp_vel_rls)
true_slip_list = [TRUE_SLIP] * n_steps

# --- Compensated trajectory replay ---
robot_ns  = Robot(initial_x=INITIAL_X, initial_y=INITIAL_Y, initial_theta=INITIAL_THETA)
robot_rls = Robot(initial_x=INITIAL_X, initial_y=INITIAL_Y, initial_theta=INITIAL_THETA)
traj_ns, traj_rls = [], []

for (vr_ns, vl_ns), (vr_rls, vl_rls) in zip(comp_vel_ns, comp_vel_rls):
    x_ns,  y_ns,  th_ns  = robot_ns.ideal_forward_kinematics(vr_ns,  vl_ns)
    x_rls, y_rls, th_rls = robot_rls.ideal_forward_kinematics(vr_rls, vl_rls)
    traj_ns.append((x_ns,  y_ns,  th_ns))
    traj_rls.append((x_rls, y_rls, th_rls))

traj_ns  = np.array(traj_ns)
traj_rls = np.array(traj_rls)

# --- Plot 1: Trajectory comparison ---
plt.figure(figsize=(8, 8))
plt.plot(reference_states[:, 0], reference_states[:, 1], linestyle=':',  label='Reference')
plt.plot(actual_states_clean[:, 0], actual_states_clean[:, 1], linestyle='-',  label='Actual (no MPC)',        color='black')
plt.plot(traj_ns[:, 0],          traj_ns[:, 1],          linestyle='-',  label='MPC (s=0)',              color='tab:orange')
plt.plot(traj_rls[:, 0],         traj_rls[:, 1],         linestyle='-.',  label='MPC + RLS',             color='tab:blue')
plt.xlabel('X (m)')
plt.ylabel('Y (m)')
plt.title('Trajectory Comparison: MPC (s=0) vs MPC + RLS')
plt.legend()
plt.gca().set_aspect('equal', adjustable='box')
plt.grid()
plt.tight_layout()
plt.show()

# --- Plot 2: Tracking error comparison ---
traj_ns_aligned  = np.vstack([[INITIAL_X, INITIAL_Y, INITIAL_THETA], traj_ns[:-1]])
traj_rls_aligned = np.vstack([[INITIAL_X, INITIAL_Y, INITIAL_THETA], traj_rls[:-1]])
error_ns  = np.linalg.norm(reference_states[:, :2] - traj_ns_aligned[:, :2],  axis=1)
error_rls = np.linalg.norm(reference_states[:, :2] - traj_rls_aligned[:, :2], axis=1)

plt.figure(figsize=(10, 4))
plt.plot(error_ns,  label='MPC (s=0)',   color='tab:orange')
plt.plot(error_rls, label='MPC + RLS',   color='tab:blue')
plt.axvline(WARMUP_STEPS, color='red', linestyle=':', linewidth=0.8, label=f'RLS warmup end (k={WARMUP_STEPS})')
plt.xlabel('Timestep')
plt.ylabel('Position error (m)')
plt.title('Tracking Error: MPC (s=0) vs MPC + RLS')
plt.legend()
plt.grid()
plt.tight_layout()
plt.show()

# --- Plot 3: RLS slip estimation ---
plt.figure(figsize=(10, 4))
plt.plot(slip_estimates, label='RLS estimate', color='tab:blue')
plt.plot(true_slip_list, linestyle='--',       label='True slip',    color='tab:red')
plt.axvline(WARMUP_STEPS, color='red', linestyle=':', linewidth=0.8, label=f'Warmup end (k={WARMUP_STEPS})')
plt.xlabel('Timestep')
plt.ylabel('Slip')
plt.title('RLS Slip Estimation')
plt.legend()
plt.grid()
plt.tight_layout()
plt.show()

print(f"MPC (s=0)  — min {error_ns.min():.3f} m, max {error_ns.max():.3f} m, mean {error_ns.mean():.3f} m")
print(f"MPC + RLS  — min {error_rls.min():.3f} m, max {error_rls.max():.3f} m, mean {error_rls.mean():.3f} m")
