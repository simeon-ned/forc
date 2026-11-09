## The question a controller answers

A controller chooses an input using available information. A robot model predicts the consequences of that input. These are separate objects, even when a learning algorithm trains the controller inside a simulator. For a deterministic continuous model write $\dot x=f(x,u)$. A sampled implementation uses $x_{k+1}=F_{\Delta t}(x_k,u_k)$ with the input held between updates.

For a robot, a convenient physical state is $(q,v,a)$: configuration, generalized velocity, and optional actuator activation. MuJoCo calls the arrays qpos, qvel, and act. Sensors produce observations $y=g(x)$; a policy receiving $y$ cannot assume access to hidden ground-truth velocity. In our first practices we deliberately use full state, and label that assumption.

## Regulation versus tracking

Regulation brings the system to a fixed equilibrium $(x^\star,u^\star)$ satisfying $f(x^\star,u^\star)=0$. Tracking follows a time-varying feasible reference $(x^\star(t),u^\star(t))$. A constant zero input is not a universal equilibrium: a hovering vehicle needs thrust to balance gravity.

Define a scalar performance measure before tuning gains. For position tracking, report RMS error in metres, peak error, control effort with units, saturation fraction, and failures. A weighted cost such as $\sum_k e_k^\top Qe_k+u_k^\top Ru_k$ is useful for optimization but does not replace those interpretable metrics. Changing units changes its numerical weights.

## Worked example: a disturbed mass

Consider $m\ddot p=u+d$, with $m=2$ kg and a constant disturbance $d=1$ N. Under proportional control $u=-k_p(p-p^\star)$ with $k_p=20$ N/m, a stationary solution has error $d/k_p=0.05$ m. Adding velocity feedback damps oscillation, but does not remove this offset. Integral action or a correctly estimated feedforward disturbance can remove it, subject to actuator limits.

An open-loop force computed from the nominal model cannot react to an unexpected disturbance. Feedback can react, but a noisy measurement or a delayed loop can also inject unwanted motion. We therefore evaluate perturbations, delay, and saturation explicitly.

## Browser experiment: joint feedback on Go2

**Predict first:** what happens to joint error and motor torque when proportional gain increases? Can joint-angle feedback alone guarantee that a floating robot balances?

The experiment applies the same PD gains to all twelve torque-controlled joints. A selected joint can receive a target offset from the home pose:

$$
\tau_i^{\mathrm{requested}}=K_p(q_i^\star-q_i)-K_d v_i.
$$

$$
\tau_i=\operatorname{clip}(\tau_i^{\mathrm{requested}},\tau_i^{\min},\tau_i^{\max}).
$$

The six floating-base velocity coordinates are unactuated. This is a joint-regulation experiment, not a locomotion or whole-body balance controller. The plots distinguish requested torque from applied actuator torque after limits.

<iframe class="lecture-experiment" src="../../experiments/go2/?lesson=control" title="Go2 joint PD feedback experiment with live state and torque plots" loading="lazy" height="1050"></iframe>

[Open the feedback experiment in a full page](../../experiments/go2/?lesson=control). Keep the initial height and disturbance schedule fixed when comparing gains. Save Run A, reset, and change one parameter. Inspect saturation and base motion as well as joint tracking. Download the CSV and settings to retain the experiment; plots retain the last fifteen simulated seconds. The browser uses MuJoCo 4.0.0, while the current Python practices use 3.13.0.

## The experimental contract

Fix the model, initial-state distribution, controller update interval, physics timestep, termination rules, and evaluation seeds. Run a baseline with the same information and force limits. Split tuning trials from final evaluation trials. Save the configuration and software version with the output, so an attractive trajectory can be reproduced.

The first practice inspects Menagerie’s UR5e, G1, and Go2 models, views them before calculation, and checks kinematics, energy, contact, and dynamics force balance. [Lecture 01](../../modules/01-modeling/notes.md) supplies the modeling theory before this practice. Later practices compare controllers under the same protocol. The external courses provide broader examples of this modeling-to-control workflow [@cmu-control] [@mit-underactuated].

## Classroom exercise

A controller reports low average tracking error, but clips its torque in 40% of steps. What evidence is missing before claiming robustness?

**Discussion guide:** inspect peak errors, constraint violations, failure cases, disturbance response, and the tested operating region. Average error alone can hide rare loss of control. A finite test suite establishes empirical performance within that suite, not a global stability proof.
