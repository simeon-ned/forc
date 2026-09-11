## PD response

$$I\ddot e+k_d\dot e+k_pe=0$$

$$\omega_n=\sqrt{k_p/I},\qquad
\zeta=\frac{k_d}{2\sqrt{Ik_p}}$$

---
## Gain example

$I=0.5$, $\omega_n=4$, $\zeta=0.8$

$k_p=8$, $k_d=3.2$

A 1 rad error requests 8 N m before gravity compensation.

---
## Saturation

The actuator may deliver less than the requested torque.

The ideal linear response then no longer describes the motion.

---
## Integral windup

Persistent error accumulates while the actuator clips.

Conditional integration can stop accumulation that worsens saturation.

---
## Sampling

Continuous-time reasoning needs a discrete-time check.

High gains, delay, and a large timestep can destabilize the implementation.

---
## Practice

Compare PD and bias compensation with identical torque limits.

Explain the tracking error and saturation traces.
