# rls_step_with_mpc.md — Key Implementation Points

## Overview

Use the working RLS estimator (with forgetting factor λ=0.98) to estimate a step-function slip in real-time and compensate wheel velocities via feedforward scaling so the compensated robot trajectory matches the target lemniscate.

No MPC — pure feedforward compensation based on the RLS slip estimate.

---

## Architecture: Feedforward Velocity Scaling

```
v_ref  ──► [FF Scaling: v_ref / (1 - s_hat)] ──► v_cmd ──► comp_robot (slip = 0)
```

The slip robot runs with the reference velocities and provides noisy theta measurements for the RLS estimator. The compensated robot runs in parallel with scaled velocities.

---

## Simulation Structure: Single Loop

Both robots advance together at every timestep. No second post-processing pass.

```
for t in range(N):
    1. slip_robot.slip = 0.05 if t >= 200 else 0.0
    2. slip_robot.forward_kinematics(v_ref)           → actual motion (reduced by slip)
    3. slip_robot.add_noise()                          → noisy theta for RLS input
    4. estimator.predict_sim_with_forgetting_factor(
           theta_noised, theta_prev, vr_ref, vl_ref, dt, lam=0.98)
    5. s_hat = clip(estimates[t][0], 0.0, 0.08)       → clamp raw estimate
    6. denom = max(1 - s_hat, 0.92)                   → floor denominator
    7. v_comp = v_ref / denom                          → compensated velocity
    8. comp_robot.forward_kinematics(v_comp)           → comp_robot.slip stays 0.0
```

**Why single-loop instead of two-pass (as in `offline_sim_step_slip.py`):**
The existing second loop skips timesteps via `if slip[timestep] <= 0.05:`, leaving gaps in the compensated trajectory. A single loop where `comp_robot` steps every timestep avoids this.

---

## Slip Clamping

| Layer | Code | Why |
|---|---|---|
| Clip | `np.clip(s_hat_raw, 0.0, 0.08)` | Rejects negative estimates and transient spikes at t=200 |
| Denominator floor | `max(1 - s_hat, 0.92)` | Caps velocity scaling at 1.087×; prevents blow-up |

No warmup suppression needed — with true slip = 0 for t < 200, the RLS estimate stays near 0, so clamping to [0, 0.08] is sufficient.

---

## Key API Calls

```python
# RLS — identical to offline_sim_step_slip.py line 40
estimator.predict_sim_with_forgetting_factor(
    theta_noised, theta_prev_noised, vr_ref, vl_ref, 0.05, lam=0.98)

# Read estimate — same indexing as offline_sim_step_slip.py line 44 (one-step lag is correct)
s_hat = float(np.clip(estimator.estimates[t][0], 0.0, 0.08))

# Feedforward compensation
denom = max(1.0 - s_hat, 0.92)
vr_comp = vr_ref / denom
vl_comp = vl_ref / denom

# Compensated robot (slip defaults to 0.0 at initialization)
comp_robot.forward_kinematics(vr_comp, vl_comp)
```

---

## Critical Design Decisions

1. **`comp_robot.slip` stays 0.0** — `forward_kinematics` with slip=0 passes velocities through unchanged, simulating a robot where slip has been perfectly cancelled by the velocity command.

2. **RLS one-step lag** — `estimates[t]` at timestep `t` is the estimate from the *previous* update call. This is unavoidable and correct; the same pattern is used in `offline_sim_step_slip.py` line 44.

3. **Do not modify `offline_sim_step_slip.py`** — keep it intact as the RLS-only demo. The new simulation lives in `simulations/sim_rls_ff_compensation.py`.

---

## Expected Behaviour

| Phase | True slip | RLS estimate | Scaling factor | Outcome |
|---|---|---|---|---|
| t < 200 | 0.0 | ≈ 0.0 | ≈ 1× | comp ≈ ref (no action needed) |
| t = 200–220 (transient) | 0.05 | rising, noisy | 1.0–1.087× | small residual drift |
| t > 220 (converged) | 0.05 | ≈ 0.05 | ≈ 1.053× | comp ≈ ref |

---

## Plots Generated

1. **Trajectories**: target (dotted), slip robot (dash-dot), compensated (solid) — primary result
2. **Slip**: true step function vs RLS clamped estimate; vertical line at t=200
3. **Tracking error**: `||comp_xy - target_xy||` vs `||slip_xy - target_xy||` over time
4. **Velocities**: `v_ref` vs `v_comp` for right and left wheels — shows scaling magnitude at slip onset

---

## Future Extension: Add MPC Feedback

Once feedforward compensation is validated, MPC can be layered on top:
- Feed `s_hat` into `mpc.s` each timestep to update the linearized dynamics model
- MPC computes additive `(delta_vr, delta_vl)` corrections on top of the FF-scaled velocities
- This corrects residual drift from the RLS transient and measurement noise
- See `tests/test_linear_mpc2.py` for the MPC integration pattern
