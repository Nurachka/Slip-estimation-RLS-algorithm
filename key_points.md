# Key Points

## mpc_no_slip.py
- Robot kinematics apply a step slip (`TRUE_SLIP = 0.05` at `SLIP_ONSET = 200`)
- MPC prediction model uses `s = 0.0` fixed throughout — completely slip-unaware
- Serves as the naive MPC baseline for comparison
- Produces: compensated trajectory, comp velocities, error states

## comparison_mpc_vs_mpc_rls.py
- Compares two approaches under identical noise (`SEED = 42`, `np.random.seed`) for a fair test
- `run_mpc_no_slip()` — MPC with `s = 0.0` always
- `run_mpc_rls()` — MPC with `s = s_hat` from online RLS, zeroed during warmup (`k < 50`)
- User updated: `TRUE_SLIP = 0.2`, `SLIP_CLIP = 0.5` (higher slip regime for stronger comparison)
- Tracking error computed from `comp_trajectory − reference_states` (true performance, not MPC internal error state)
- Theta error is angle-wrapped with `arctan2`
- 4 comparison figures: trajectory, x/y/θ error (3 stacked subplots), position error magnitude, velocity corrections

## Design decisions across both files
- RLS warmup guard: `s_hat` forced to `0.0` for `k < WARMUP_STEPS = 50`
- Compensated trajectory replayed via `ideal_forward_kinematics` (no slip in replay robot)
- Slip onset marked with vertical grey dotted line at `k = 200` on all time-series plots
