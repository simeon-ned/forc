## Compensation as a model-based feedforward term

Consider a fully actuated robot without contact:
$$M(q)\dot v+c(q,v)=u+\tau_{\rm passive}.$$
Here $u$ means generalized motor force, which equals ctrl only in our unit-gear direct-drive teaching model. Gravity compensation adds the gravity contribution to a feedback controller. Bias compensation adds all of $c(q,v)$, including velocity-dependent terms.

At zero velocity, c is the gravity bias for a conventional mechanical model. With nonzero velocity, it is generally not just gravity. If damping is already a passive force in the simulator, include it explicitly in any exact cancellation argument.

## Computed torque

For hinge coordinates with $e=q-q^\star$, define
$$a^\star=\ddot q^\star-K_d(v-v^\star)-K_pe.$$
Then request
$$u=M(q)a^\star+c(q,v)-\tau_{\rm passive}.$$
If the model is exact and the actuator realizes this force without saturation, the error obeys $\ddot e+K_d\dot e+K_pe=0$. The apparent decoupling comes from canceling the coupled dynamics, not from assuming the mass matrix is diagonal.

Underactuated robots cannot realize an arbitrary generalized force. With $u=B(q)\eta$, solve an allocation problem or redesign the desired acceleration. A floating-base robot with no contact cannot independently command its base acceleration and every joint acceleration.

## Worked pendulum

For $I\ddot q+b\dot q+mg\ell\sin q=u$, the bias is $mg\ell\sin q$ and the passive force is $-b\dot q$. The computed-torque command becomes
$$u=I(-k_pe-k_d\dot e)+mg\ell\sin q+b\dot q$$
for a fixed target. The positive damping-cancellation term is easy to get wrong. Without it, physical damping remains in the closed loop; that may be useful, but the claimed error equation must change.

If the true inertia is twice the model inertia, the cancellation is imperfect. Run the model-mismatch experiment before presenting exact-model behavior as robustness.

## MuJoCo force queries

After mj_forward, retrieve c from qfrc_bias and the passive force from qfrc_passive. Use [mj_fullM](https://mujoco.readthedocs.io/en/stable/APIreference/APIfunctions.html#mj-fullm) to materialize the dense mass matrix when needed for these small examples.

[mj_inverse](https://mujoco.readthedocs.io/en/stable/APIreference/APIfunctions.html#mj-inverse) answers a related query after qacc is set. With constraints and passive forces present, interpreting its result requires the documented decomposition. It is not automatically an actuator command.

## Exercise and practice

Practice 2 compares PD, bias-compensated PD, and computed torque on the same two-link arm. Plot error and saturation, then reduce the motor limit.

**Check:** Why can adding exact gravity compensation worsen an experiment? Possible causes include a wrong sign, wrong frame, double-counted compensation, stale bias data, or actuator saturation. Compensation also consumes some of the available motor force. Diagnose these before changing the feedback gains.
