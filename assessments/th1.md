## TH1: constrained quadrotor motion

**Weight:** 25% of the course. **Suggested release:** after Lecture 6. **Prerequisites:** Practices 3 and 4. **Estimated effort:** 8–12 hours. The instructor sets the submission date.

### Question

When does a constrained predictive controller improve on local LQR for a quadrotor, and what does that improvement cost computationally?

### Plant and conventions

Use the full free-joint vehicle from src/forc/models.py, not only the vertical-flight approximation. Keep scalar-first quaternions and a 12-dimensional local state error. Controls are four nonnegative rotor thrusts with the model's moment coefficients. Declare mass, gravity, timestep, policy rate, thrust limits, and target poses.

### Required work

1. Reproduce the hover equilibrium and local LQR baseline. Verify transition dimensions and the equilibrium residual. Explain why subtracting seven qpos values does not produce six configuration-error coordinates.
2. Construct a feasible rest-to-rest reference between two nearby positions. Implement direct shooting or transcription with explicit dynamics constraints. Penalize orientation using tangent errors and include rotor limits. Report the final optimization residual and whether the solution actually reaches the target.
3. Implement receding-horizon tracking. A sequential linear MPC around the reference is acceptable. Explain how you construct A, B, affine defects, terminal cost, and control bounds. Report the solver status and a declared fallback.
4. Compare with clipped hover LQR and, if used, a trajectory-tracking baseline. Use at least ten paired initial conditions or disturbances. Include one case where constraints become active and one deliberately infeasible or unrecoverable case.
5. Perform one ablation: horizon, timestep, terminal penalty, or model mismatch. Keep other factors fixed.

### Evidence

Report position RMS and peak error in metres, attitude error in radians, thrust saturation, constraint violations, solver failures, and median/95th-percentile compute time. Save configuration and trajectory files. Explain the difference between predicted and realized constraints. A collision penalty is not a hard collision-avoidance guarantee.

### Rubric

- 15%: model, notation, equilibrium, and derivative checks.
- 25%: feasible reference formulation and numerical verification.
- 30%: predictive controller implementation and constraint handling.
- 20%: fair comparison, ablation, and failure interpretation.
- 10%: reproducible commands, configuration, and readable report.

### Submission

Submit code, a YAML experiment manifest, numerical results, and a report of at most six pages excluding references. A short animation is optional supporting evidence. State which external ideas or code you used and their licenses. Rerunning the one-axis reference practice alone is not a complete TH1 submission.
