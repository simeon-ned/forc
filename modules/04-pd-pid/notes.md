## PD as state feedback

For one inertial joint, approximate $I\ddot q=u$ near an operating point. With $e=q-q^\star$ and $u=-k_pe-k_d\dot e$,
$$I\ddot e+k_d\dot e+k_pe=0.$$
The characteristic polynomial gives natural frequency $\omega_n=\sqrt{k_p/I}$ and damping ratio $\zeta=k_d/(2\sqrt{Ik_p})$. Positive gains stabilize this ideal continuous scalar system. A coupled robot, sampled controller, or delayed measurement requires additional analysis.

The velocity term dissipates motion. The proportional term creates a restoring action. Increasing both gains can shorten the ideal response while demanding larger torques and making the implementation more sensitive to delay and noise.

## Worked gain choice

For $I=0.5$ kg m$^2$, $\omega_n=4$ rad/s and $\zeta=0.8$, choose $k_p=I\omega_n^2=8$ N m/rad and $k_d=2\zeta I\omega_n=3.2$ N m s/rad. A 1 rad initial error requests 8 N m even before including gravity. If the actuator limit is 3 N m, the intended second-order response is immediately unavailable.

This calculation is a starting point for tuning, not a guarantee for the full robot. Inspect the actual coupled motion and saturation trace.

## Integral action

For the same error convention, let $\dot z=e$ and $u_{\rm raw}=-k_pe-k_dv-k_iz$. A constant disturbance can be rejected at equilibrium because $z$ supplies the required steady force. But if the actuator clips while the error persists, $z$ keeps growing. Once the target becomes reachable, the accumulated integral can cause a large overshoot.

A simple conditional-integration rule freezes the integral when saturation and the proposed integral update would drive the command farther outside the limit. Back-calculation is another option. State the sign convention and test it on both positive and negative saturation. Reset or deliberately preserve the integrator when resetting an episode; either choice changes the experiment.

## Robot-specific details

Joint-space PD is convenient for independent hinge joints. For a ball or free joint, compute a tangent-space position error instead of subtracting qpos arrays. A gain on orientation error should multiply three rotational components.

A motor command equals joint torque only for the declared direct, unit-gear actuator. With transmissions, activation dynamics, or position actuators, ctrl has a different interpretation. Inspect qfrc_actuator before interpreting a gain numerically.

## Exercise

For a unit mass, apply semi-implicit Euler with $k_p=100$, $k_d=0$, and $\Delta t=0.25$ s. Is positive stiffness enough to make the discrete trajectory stable?

**Answer check:** the undamped symplectic Euler oscillator requires $\Delta t\sqrt{k_p/m}<2$ for bounded generic trajectories. Here the product is 2.5, so the sampled numerical system is unstable despite stable continuous oscillations. The modeling course derives this limitation.

The practice compares PD with bias-compensated control under the same torque limit. Use the failure cases to decide whether poor tracking comes from missing dynamics, insufficient actuation, or a timing problem.
