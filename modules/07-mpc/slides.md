## Receding horizon

Solve from the measured state.

Apply the first control.

Shift the plan and solve again at the next tick.

---
## Constrained objective

$$\min \sum_{k=0}^{N-1}(x_k^\top Qx_k+u_k^\top Ru_k)+x_N^\top P_fx_N$$

Dynamics, input limits, and state limits define feasibility.

---
## Stopping distance

$v=2$ m/s, maximum braking $a=1$ m/s$^2$

$$d_{\rm stop}=\frac{v^2}{2a}=2\ {\rm m}$$

A wall 1 m away makes collision avoidance infeasible.

---
## Solver outcome

Infeasible model, short horizon, and numerical failure are different diagnoses.

Record status and residuals.

---
## Vertical-flight practice

The core example controls height and vertical speed.

Thrust limits become acceleration bounds around hover.

The take-home extends to attitude dynamics.

---
## Comparison

MPC and clipped LQR use identical limits.

Compare violations, tracking, and computation time.

Reading: [@mit-trajopt]
