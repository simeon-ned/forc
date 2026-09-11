## A complete robot controller

A useful robot-control system usually contains more than one algorithm. A planner selects a task or reference, an estimator reconstructs the state, a feedback controller tracks the reference, and an actuator interface enforces the hardware contract. Learned policies can replace or assist individual layers. The interfaces must still specify units, frames, update rates, and failure behavior.

For legged robots, contact changes the feasible forces and dynamics. During flight, the base cannot produce arbitrary translational acceleration through internal joint torques. During stance, ground reaction forces provide additional authority, subject to friction and unilateral contact. A policy that ignores the contact mode can request physically impossible motion.

## A locomotion case study

Consider a floating-base robot tracking a commanded forward speed. A model-based stack may plan center-of-mass motion and foot placement, then solve for feasible contact forces and joint torques. A learned policy may map proprioception and commands to joint targets, with lower-level PD control producing torque.

These systems expose different action spaces. Comparing a learned joint-target policy with a torque controller requires including the lower-level actuator dynamics and PD gains. Equal policy rates do not imply equal closed-loop bandwidth.

## Worked interface audit

A policy trained at 50 Hz emits joint targets. The simulator runs at 1 kHz and a PD servo evaluates every physics step. Deploying the policy at 50 Hz while evaluating PD only at that same rate changes the control system. The policy weights can be identical while behavior changes substantially.

The same audit applies to observation filtering, frame conventions, torque clipping, and delayed measurements. Record the full loop rather than only the policy network.

## Project experiment

Choose a bounded task: cart-pole recovery, constrained flight, arm tracking with payload changes, or a contact-rich manipulation problem. Declare one main question, a baseline, a candidate improvement, and measurable success criteria. Keep the first evaluation suite small enough to run reliably on a CPU.

Use paired initial conditions across methods. Include nominal and deliberately perturbed cases, at least one ablation, and a failure analysis. Report every tested seed and failed episode. A video is supporting evidence; the configuration, code, and numerical results establish reproducibility.

## Limits of the claims

A simulation comparison can show improved performance on the chosen test distribution. It cannot alone establish hardware safety, global stability, or superiority across robots. Explain model simplifications and the operating region. For contact tasks, vary friction and solver settings separately from controller gains.

The requested MIT material provides richer underactuated and manipulation examples, while MJX offers a path to larger simulation batches [@mit-underactuated] [@mit-manipulation] [@mujoco-mjx].

## Final discussion

Which part of your system uses a model, which part uses data, and which part handles constraints? A strong project answers this precisely, explains the observed tradeoffs, and leaves a reusable experiment that someone else can run. The project practice supplies a report checklist and reproducibility command rather than a pre-solved research result.
