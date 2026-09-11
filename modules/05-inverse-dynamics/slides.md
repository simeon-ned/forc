## Model-based compensation

$$M\dot v+c=u+\tau_{\rm passive}$$

Gravity compensation uses the gravitational bias.

Bias compensation also includes velocity-dependent inertial terms.

---
## Desired acceleration

$$a^\star=\ddot q^\star-K_d(v-v^\star)-K_p(q-q^\star)$$

The reference must specify position, velocity, and acceleration consistently.

---
## Computed torque

$$u=Ma^\star+c-\tau_{\rm passive}$$

Exact cancellation assumes an accurate model and sufficient actuation.

---
## Passive damping

For $\tau_{\rm passive}=-bv$, cancellation contributes $+bv$.

Leaving physical damping in the loop changes the error equation.

---
## MuJoCo connection

qfrc_bias, qfrc_passive, and mj_fullM expose the required terms.

mj_inverse requires a desired qacc and careful force interpretation.

---
## Failure analysis

Reduce the actuator limit.

Does compensation still improve tracking?

Reading: [@mit-multibody]
