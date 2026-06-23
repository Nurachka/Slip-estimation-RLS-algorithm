# Slip Estimation Evaluation Methods

## Quantitative Metrics

### 1. Slip Estimation RMSE and Bias
The most direct measure. Since `S_ACTUAL` is known, compute:
- `RMSE = sqrt(mean((s_hat - s_actual)²))` — overall estimation accuracy
- `Bias = mean(s_hat - s_actual)` — whether the estimator systematically over/underestimates (this is what caught the SLIP_CLIP bug)
- Computed over the full run AND over the steady-state window (e.g., last 100 steps after convergence)

### 2. Convergence Time
Number of steps (or seconds) until `|s_hat - s_actual| < threshold` for the first time and stays there. A threshold of 10–20% of true slip (e.g., ±0.01 for `s=0.1`) is sensible. Gives a single number to compare tuning choices (P0, R, forgetting factor).

### 3. Heading Error Over Time
Since slip acts exclusively through `omega = (1-s)*(vr-vl)/L`, it corrupts heading first and position second. Plotting `theta_actual - theta_ref` separately from position error shows whether the RLS is fixing the right thing. Position error is a lagging, integrated effect — heading error is more directly coupled to slip.

### 4. Innovation Autocorrelation
For a statistically optimal estimator, the innovation sequence `e_k = y_k - C*s_hat_k` should be white noise (zero autocorrelation at all lags > 0). If it is correlated, the model is mismatched — e.g., the noise covariance R is wrong, or the slip model does not capture the true dynamics. The `INNOVATION` list is already collected; plotting its autocorrelation (`np.correlate`) is a simple consistency check.

---

## Qualitative Methods

### 5. Confidence Band Plot
Plot `s_hat ± 2*sqrt(P_k)` alongside the true slip line. A well-calibrated RLS should contain the true value within this 2-sigma band ~95% of the time. If the band shrinks to near-zero while the estimate is still biased (which is what happened with the clip), you can see visually that the estimator is overconfident and wrong. The `estimationerrorcovariancematrices` list is already collected.

### 6. Innovation Sequence Plot
Already coded but commented out (Figure 5). Uncomment it. Visually, it should look like zero-mean noise with no obvious trend or periodicity. Systematic patterns (e.g., spikes at the center crossings where C≈0, or a persistent sign after t=15) confirm specific failure modes.

### 7. Phase Portrait: Slip Error vs Position Error
Scatter plot of `(s_hat - s_actual)` on x-axis vs position error on y-axis, colored by time. This shows the coupling: large slip error should correlate with large position error, and as the RLS converges (moving left on the x-axis), position error should reduce (moving down on y). A tight diagonal cluster means MPC+RLS is working; a flat horizontal cluster means MPC is correcting position error even with a bad slip estimate (relying more on feedback than feedforward model correction).

---

## Priority Recommendation

For the current setup, add in this order:
1. **Slip RMSE + bias** — numbers, immediately tells you if the estimator is working
2. **Confidence band plot** — catches clip/overconfidence issues visually
3. **Heading error** — shows the mechanism, not just the symptom
4. **Convergence time** — gives a single number for tuning comparisons

The innovation autocorrelation and phase portrait are more for deeper diagnosis once you want to validate the statistical model assumptions.
