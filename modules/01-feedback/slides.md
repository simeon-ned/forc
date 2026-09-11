## Feedback loop

The plant evolves under the applied input.

The controller uses observations to choose the next input.

An estimator can supply quantities that sensors do not measure directly.

---
## Robot state

$$x=(q,v,a),\qquad y=g(x)$$

MuJoCo: qpos, qvel, act.

The first labs assume full-state observation.

---
## Regulation and tracking

$$f(x^\star,u^\star)=0$$

A hover equilibrium requires nonzero thrust.

A tracking reference must respect the dynamics.

---
## Disturbed mass

$$m\ddot p=-k_p(p-p^\star)-k_d\dot p+d$$

At rest, the position offset is $d/k_p$.

For $d=1$ N and $k_p=20$ N/m: 5 cm.

---
## Evaluation

RMS and peak error, force limits, failures.

Same initial states and disturbance schedule for every controller.

---
## Discussion

A policy has low mean error and frequent saturation.

What would you test next?

Further reading: [@cmu-control] [@mit-underactuated]
