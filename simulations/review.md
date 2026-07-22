# Review: Equations and RLS Approach for `offline_fixed_slip_rls.py`

This document explains the mathematics behind
[`offline_fixed_slip_rls.py`](offline_fixed_slip_rls.py): how the differential-drive
robot is modelled, how a **constant ("fixed") longitudinal slip** is estimated online
with **Recursive Least Squares (RLS)** from noisy heading measurements, and how that
estimate is used to pre-compensate the feedforward wheel velocities so the robot tracks a
lemniscate trajectory.

Equations below use the same symbol names as the code so the two can be cross-referenced
(see the [notation table](#7-notation--codesymbol-map) at the end).

---

## 1. Overview

The goal is to estimate a single scalar wheel-slip factor $s$ from noisy heading
measurements and use it to scale the commanded (feedforward) wheel velocities, so that the
*actual* slipped robot follows the target trajectory.

The processing pipeline in the script is:

```
lemniscate_trajectory.csv
      │  (Analysis.read_csv)
      ▼
Feedforward.vel_at_timestep  ──►  (v_R, v_L)   commanded wheel velocities
      ▼
Robot.forward_kinematics(v_R, v_L)             true pose WITH slip
      ▼
Robot.add_noise()                              noisy measured heading θ̃
      ▼
RecursiveLeastSquares.predict_sim(...)         slip estimate ŝ_k
      ▼
Compensator.modify_velocities(ŝ, df)           v' = v / (1 − ŝ)
      ▼
re-simulate  ──►  compensated trajectory
```

"Fixed slip" means the true slip is a constant (it does not vary in time), which is why the
plain RLS recursion is used rather than the forgetting-factor variant — see
[§4](#4-the-rls-recursion).

---

## 2. Robot model

Source: [`../mathematical_simulator_class/robot.py`](../mathematical_simulator_class/robot.py),
constants in [`../mathematical_simulator_class/config.py`](../mathematical_simulator_class/config.py).

The robot is a differential-drive vehicle with wheel base $L$ and fixed integration step
$\Delta t$. Longitudinal wheel slip is modelled as a multiplicative loss factor
$s \in [0,1)$ applied to both wheels:

| Symbol | Value | Source |
|--------|-------|--------|
| $L$ (`WHEEL_BASE`) | $0.5\ \text{m}$ | `config.py` |
| $\Delta t$ (`TIMESTEP`) | $0.05\ \text{s}$ | `config.py` |
| $s$ (`Robot.slip`) | $0.19$ | `robot.py` |
| $\sigma_{\text{pos}}$ (`NOISE_STD_POSITION`) | $0.025\ \text{m}$ | `config.py` |
| $\sigma_{\theta}$ (`NOISE_STD_ORIENTATION`) | $0.00436\ \text{rad}$ | `config.py` |

### Kinematics with slip

The body linear and angular velocities are the ideal differential-drive expressions scaled
by $(1-s)$ (`forward_kinematics`, lines 31–32):

$$
v = (1-s)\,\frac{v_R + v_L}{2},
\qquad
\omega = (1-s)\,\frac{v_R - v_L}{L}.
$$

The pose is propagated with a first-order (Euler) integration using the *previous* heading
(lines 34–36):

$$
\begin{aligned}
x_k &= x_{k-1} + v\,\cos(\theta_{k-1})\,\Delta t, \\
y_k &= y_{k-1} + v\,\sin(\theta_{k-1})\,\Delta t, \\
\theta_k &= \theta_{k-1} + \omega\,\Delta t.
\end{aligned}
$$

(`ideal_forward_kinematics` is the same with $s=0$; it is not used in this script.)

### Measurement noise

`add_noise` (lines 57–65) adds zero-mean Gaussian noise to the latest pose:

$$
\tilde{x} = x + \eta_x,\quad
\tilde{y} = y + \eta_y,\quad
\tilde{\theta} = \theta + \eta_\theta,
$$

with $\eta_x,\eta_y \sim \mathcal{N}(0,\sigma_{\text{pos}}^2)$ and
$\eta_\theta \sim \mathcal{N}(0,\sigma_{\theta}^2)$. Only the noisy heading
$\tilde{\theta}$ feeds the estimator.

### Note on the initial pose

The robot is initialised at $(x_0,y_0,\theta_0) = (1.0,\ 0.0,\ 1.5786\ldots)$ rather than at
the origin (script lines 13–14, 16). Starting at $(1,0)$ with $\theta_0 \approx \pi/2$ places
the robot at the correct point on the lemniscate with the correct initial heading, so the
figure-eight is generated in its intended *horizontal* orientation instead of a vertical one.

---

## 3. Deriving the slip measurement model

The estimator observes the **change in heading** between consecutive steps, which isolates
the slip parameter. Starting from the heading update above:

$$
\theta_k - \theta_{k-1} = \omega\,\Delta t
= (1-s)\,\frac{v_R - v_L}{L}\,\Delta t.
$$

Define the **ideal (no-slip) yaw rate** and the scalar **regressor** $C_k$:

$$
\omega_z = \frac{v_R - v_L}{L}
\quad(\texttt{angular\_vel\_z}),
\qquad
C_k = \Delta t\,\omega_z
\quad(\texttt{C}).
$$

Both are computed from the *commanded* wheel velocities, which are known exactly. The
measured heading increment is then

$$
\Delta\theta_k = C_k\,(1-s) = C_k - C_k\,s,
$$

which rearranges into a form that is **linear in the unknown slip** $s$:

$$
\boxed{\; C_k - \Delta\theta_k = C_k\,s \;}
$$

This is a standard linear-in-parameter measurement model

$$
y_k = C_k\,s + \text{noise},
\qquad
y_k = C_k - \Delta\theta_k,
$$

where the "measurement" $y_k = C_k - \Delta\theta_k$ combines the known regressor $C_k$ with
the observed heading increment, the regressor is $C_k$, and the single unknown scalar
parameter is the slip $s$. The measurement noise enters through $\Delta\theta_k$, because
$\Delta\theta_k$ is formed from the *noised* heading.

### Wrap-safe heading difference

$\Delta\theta_k$ is computed with an `atan2` wrap so it always lands in $(-\pi,\pi]$
(lines 49–50):

$$
\Delta\theta_k = \operatorname{atan2}\!\big(\sin(\tilde\theta_k - \tilde\theta_{k-1}),\;
\cos(\tilde\theta_k - \tilde\theta_{k-1})\big).
$$

This prevents a $2\pi$ jump (when the raw heading crosses $\pm\pi$) from being mistaken for a
huge angular change, which would otherwise corrupt the estimate. The two headings used are
the noisy measurements $\tilde\theta_k$ and $\tilde\theta_{k-1}$ (`theta_noised`,
`theta_previous_noised`).

---

## 4. The RLS recursion

Source: `RecursiveLeastSquares.predict_sim` in
[`../mathematical_simulator_class/recursive_least_square.py`](../mathematical_simulator_class/recursive_least_square.py),
lines 44–80.

Given the linear model $y_k = C_k s + \text{noise}$, RLS updates the estimate $\hat{s}_k$ and
its error covariance $P_k$ recursively at each time step. In this scalar case ($s$ is a single
number) the matrices are $1\times1$, but the code keeps the general matrix form. Each equation
below is paired with its code line.

**1. Innovation covariance** (line 55):

$$
L_k = R + C_k\,P_{k-1}\,C_k^{\mathsf T}.
$$

**2. Kalman / RLS gain** (line 59):

$$
K_k = P_{k-1}\,C_k^{\mathsf T}\,L_k^{-1}.
$$

**3. Innovation (correction term)** (line 62):

$$
e_k = \underbrace{(C_k - \Delta\theta_k)}_{y_k} - C_k\,\hat{s}_{k-1}.
$$

This is the difference between the measurement $y_k$ and its prediction $C_k\hat{s}_{k-1}$
using the previous estimate.

**4. State (slip) update** (line 65):

$$
\hat{s}_k = \hat{s}_{k-1} + K_k\,e_k.
$$

**5. Covariance update** (lines 68–69):

$$
P_k = (I - K_k C_k)\,P_{k-1}.
$$

### Initialization and tuning

The estimator is constructed on script line 22:

```python
estimator = RecursiveLeastSquares(s0=np.array([0.0]),
                                  P0=10*np.eye(1,1),
                                  R=0.00436*np.eye(1,1))
```

- $\hat{s}_0 = 0$ — start from "no slip"; we have no prior knowledge of the slip magnitude.
- $P_0 = 10$ — a **large** initial covariance encodes low confidence in $\hat{s}_0$. This makes
  the early gains large, so the estimate adapts quickly in the first steps. As data accumulate,
  $P_k$ shrinks and the estimate settles (the script plots this decay).
- $R = 0.00436$ — the measurement-noise covariance, tuned to the heading-noise level
  $\sigma_\theta$. **Implementation note:** $R$ is set to the *standard deviation* value
  $\sigma_\theta = 0.00436$, not its variance $\sigma_\theta^2 \approx 1.9\times10^{-5}$. This is
  a tuning choice in the code as written; a strictly statistical interpretation of $R$ would use
  the variance. It is flagged here rather than silently changed, because it affects how heavily
  the filter trusts each measurement (larger $R$ ⇒ smoother but slower convergence).

### Why the plain recursion (no forgetting factor)

The same class provides `predict_sim_with_forgetting_factor`, which is identical except the
covariance update is inflated by $1/\lambda$ (line 93):

$$
P_k = (I - K_k C_k)\,P_{k-1}\,\frac{1}{\lambda}, \qquad 0<\lambda\le 1.
$$

A forgetting factor $\lambda<1$ keeps $P_k$ from collapsing, letting the estimate keep tracking
a **time-varying** parameter. Because this scenario has a **fixed** slip, no forgetting is
needed ($\lambda = 1$): the estimate should converge to the constant and stay there, so the
script calls the plain `predict_sim`. The forgetting variant is the tool for the sinusoidal /
time-varying-slip simulations.

---

## 5. Compensation

Source: [`../mathematical_simulator_class/compensator.py`](../mathematical_simulator_class/compensator.py).

Once the estimate has converged, `modify_velocities` scales both commanded wheel velocities by
$1/(1-\hat{s})$ (lines 23–24):

$$
v_R' = \frac{v_R}{1-\hat{s}},
\qquad
v_L' = \frac{v_L}{1-\hat{s}}.
$$

Feeding these compensated velocities back through the slipped kinematics cancels the slip: the
$(1-s)$ factor in `forward_kinematics` is undone by the $1/(1-\hat{s})$ pre-scaling. For the
linear velocity,

$$
(1-s)\,v' = (1-s)\,\frac{v}{1-\hat{s}} \;\approx\; v
\qquad\text{when } \hat{s}\approx s,
$$

and identically for $\omega$. The residual error is governed by how close $\hat{s}$ is to the
true $s$.

The script uses the **final** estimate `slip[-1]` (line 82) as a single, constant compensation
applied to the whole trajectory — appropriate here precisely because the slip is fixed.

---

## 6. What the script plots and reports

For reference, the driver produces:

- **Trajectories** — target vs. slipped (uncompensated) vs. noised vs. compensated, to show that
  compensation recovers the target path (lines 49–99).
- **Slip convergence** — estimated $\hat{s}_k$ over time against a dashed reference line drawn at
  `0.1` (lines 133–140). Note that `Robot.slip` is actually `0.19`, so the reference line and the
  true slip do **not** coincide in the current code — the line at `0.1` is a plotting constant,
  not the true value. This mismatch is worth reconciling if the plot is meant to show convergence
  to the true slip.
- **Covariance decay** — $P_k$ over time, illustrating growing confidence (lines 150–156).
- **Kalman gain decay** — $K_k$ over time, shrinking as $P_k$ shrinks (lines 158–164).
- **Tracking error** — position error between the compensated and target trajectories, plus
  printed min/max/mean (lines 118–130).
- **Velocity comparison** — reference vs. compensated left/right wheel velocities
  (lines 176–202).

---

## 7. Notation ↔ code/symbol map

| Math symbol | Meaning | Code name | Location |
|-------------|---------|-----------|----------|
| $s$ | true slip factor | `Robot.slip` | `robot.py:12` |
| $\hat{s}_k$ | estimated slip | `estimator.estimates[k]` / `slip` | `recursive_least_square.py`, script |
| $L$ | wheel base | `WHEEL_BASE` / `self.L` | `config.py`, `recursive_least_square.py:37` |
| $\Delta t$ | time step | `delta_t` (`0.05`) | `config.py`, script line 37 |
| $\omega_z$ | ideal yaw rate | `angular_vel_z` | `recursive_least_square.py:52` |
| $C_k$ | regressor $\Delta t\,\omega_z$ | `C` | `recursive_least_square.py:53` |
| $\Delta\theta_k$ | wrapped heading increment | `theta_diff` | `recursive_least_square.py:49-50` |
| $y_k$ | measurement $C_k-\Delta\theta_k$ | `(C - theta_diff)` | `recursive_least_square.py:62` |
| $L_k$ | innovation covariance | `L_matrix` | `recursive_least_square.py:55` |
| $K_k$ | RLS/Kalman gain | `gain_matrix` | `recursive_least_square.py:59` |
| $e_k$ | innovation | `error` | `recursive_least_square.py:62` |
| $P_k$ | estimation error covariance | `estimationErrorCovarianceMatrices[k]` | `recursive_least_square.py` |
| $R$ | measurement-noise covariance | `R` (`0.00436`) | script line 22 |
| $\sigma_\theta$ | heading noise std | `NOISE_STD_ORIENTATION` | `config.py` |
| $\lambda$ | forgetting factor (unused here) | `lam` | `recursive_least_square.py:82` |
