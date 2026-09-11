## Planning repeatedly

Model predictive control solves a finite-horizon problem from the latest measured state, applies the first input, then solves again. Prediction makes constraints visible before they become active. Replanning provides feedback when the measured trajectory differs from the prediction.

For a linear model,
$$\min_{x_{0:N},u_{0:N-1}}
x_N^\top P_fx_N+\sum_{k=0}^{N-1}(x_k^\top Qx_k+u_k^\top Ru_k)$$
subject to $x_0=\hat x$, $x_{k+1}=Ax_k+Bu_k$, input bounds, and optional state inequalities. With positive semidefinite state costs, positive definite input cost, and linear constraints, this is a convex quadratic program.

## Worked stopping-distance example

A cart moves at 2 m/s toward a wall 1 m away. Maximum braking acceleration is 1 m/s$^2$. Even with instantaneous control, its stopping distance is $v^2/(2a)=2$ m. An optimizer cannot create a feasible collision-free trajectory from this state. Increasing the wall penalty does not change the physical limit.

This distinction matters in controller reports: infeasibility can mean the state is already unrecoverable, the horizon is too short, the terminal condition is too strict, or the numerical solver failed. Log solver status and constraint residuals to distinguish them.

## Condensing and implementation

For small linear examples, eliminate the predicted states by repeated substitution and optimize only the control sequence. A general constrained solver can demonstrate the idea, though specialized QP solvers are preferable for larger real-time systems. Shift the previous solution to warm-start the next solve. Report solve-time distributions rather than only an average.

The practice solves a constrained vertical-flight problem. Position and velocity form a double-integrator state around hover. Input is acceleration relative to gravity, with bounds derived from thrust and mass. This is explicitly a one-axis quadrotor model. The full attitude/thrust model from Practice 3 becomes the take-home extension.

## Feasibility and safety

Soft constraints add penalized slack and can keep a numerical problem solvable. They also permit the modeled violation. Never label a softened collision constraint as a guarantee of collision avoidance. Terminal costs and invariant terminal sets can support stability and recursive-feasibility results under stated assumptions; a finite horizon by itself does not provide those results.

If a solve fails, use a declared fallback that respects actuator limits and terminate when the safe operating envelope is lost. Reusing an arbitrary stale command is not a general safe fallback.

## Exercise and comparison

Compare MPC with LQR followed by clipping, using identical initial states, bounds, and simulation rates. Plot predicted versus realized motion and count constraint violations. Explain what changes when the horizon doubles.

**Answer guide:** a longer horizon may anticipate braking earlier, but increases computation. It does not automatically improve a poor model or rescue an already infeasible state. See the trajectory-optimization notes for related finite-horizon formulations [@mit-trajopt].
