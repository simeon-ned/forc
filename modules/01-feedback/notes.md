## The question a controller answers

A controller chooses an input using available information. A robot model predicts the consequences of that input. These are separate objects, even when a learning algorithm trains the controller inside a simulator. For a deterministic continuous model write $\dot x=f(x,u)$. A sampled implementation uses $x_{k+1}=F_{\Delta t}(x_k,u_k)$ with the input held between updates.

For a robot, a convenient physical state is $(q,v,a)$: configuration, generalized velocity, and optional actuator activation. MuJoCo calls the arrays qpos, qvel, and act. Sensors produce observations $y=g(x)$; a policy receiving $y$ cannot assume access to hidden ground-truth velocity. In our first practices we deliberately use full state, and label that assumption.

## Regulation versus tracking

Regulation brings the system to a fixed equilibrium $(x^\star,u^\star)$ satisfying $f(x^\star,u^\star)=0$. Tracking follows a time-varying feasible reference $(x^\star(t),u^\star(t))$. A constant zero input is not a universal equilibrium: a hovering vehicle needs thrust to balance gravity.

Define a scalar performance measure before tuning gains. For position tracking, report RMS error in metres, peak error, control effort with units, saturation fraction, and failures. A weighted cost such as $\sum_k e_k^\top Qe_k+u_k^\top Ru_k$ is useful for optimization but does not replace those interpretable metrics. Changing units changes its numerical weights.

## Worked example: a disturbed mass

Consider $m\ddot p=u+d$, with $m=2$ kg and a constant disturbance $d=1$ N. Under proportional control $u=-k_p(p-p^\star)$ with $k_p=20$ N/m, a stationary solution has error $d/k_p=0.05$ m. Adding velocity feedback damps oscillation, but does not remove this offset. Integral action or a correctly estimated feedforward disturbance can remove it, subject to actuator limits.

An open-loop force computed from the nominal model cannot react to an unexpected disturbance. Feedback can react, but a noisy measurement or a delayed loop can also inject unwanted motion. We therefore evaluate perturbations, delay, and saturation explicitly.

## The experimental contract

Fix the model, initial-state distribution, controller update interval, physics timestep, termination rules, and evaluation seeds. Run a baseline with the same information and force limits. Split tuning trials from final evaluation trials. Save the configuration and software version with the output, so an attractive trajectory can be reproduced.

The first practice inspects a simple MuJoCo model, advances it without a viewer, and verifies the dynamics force balance. Later practices compare controllers under the same protocol. The external courses provide broader examples of this modeling-to-control workflow [@cmu-control] [@mit-underactuated].

## Classroom exercise

A controller reports low average tracking error, but clips its torque in 40% of steps. What evidence is missing before claiming robustness?

**Discussion guide:** inspect peak errors, constraint violations, failure cases, disturbance response, and the tested operating region. Average error alone can hide rare loss of control. A finite test suite establishes empirical performance within that suite, not a global stability proof.
