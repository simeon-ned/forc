## Continuous equations and a discrete experiment

Use the course convention
$$M(q)\dot v+c(q,v)=\tau+J^\top f.$$
The bias $c$ contains gravity and velocity-dependent inertial effects. Other texts may call it $h$. The applied generalized force $\tau$ includes actuator, passive, and externally applied forces. The constraint contribution $J^\top f$ stays separate. See the notation page before translating code from another engine.

MuJoCo's model describes masses, joints, geometry, and actuators. Its data object contains a particular state and derived quantities. Changing qpos does not automatically refresh every Jacobian or force array. Call [mj_forward](https://mujoco.readthedocs.io/en/stable/APIreference/APIfunctions.html#mj-forward) when you need consistent derived quantities without advancing time.

## A reliable control step

At each controller tick: refresh the current state quantities, compute the command, clip it to the declared actuator range, and apply it for a fixed number of physics steps. [mj_step](https://mujoco.readthedocs.io/en/stable/APIreference/APIfunctions.html#mj-step) advances the simulation. A 500 Hz physics loop and a 100 Hz controller imply five physics steps per command. They do not imply five policy evaluations.

~~~python
mujoco.mj_forward(model, data)
data.ctrl[:] = policy(model, data)
for _ in range(control_substeps):
    mujoco.mj_step(model, data)
mujoco.mj_forward(model, data)  # quantities at the new state
~~~

Avoid silently running the policy at every integrator stage through a callback when comparing against a zero-order-hold controller. The discrete map being differentiated or optimized must match the map being executed.

## Worked diagnostic: force balance

For a contact-free model with no Cartesian applied forces, form
$$r=M\dot v+c-\tau_{\rm actuator}-\tau_{\rm passive}
-\tau_{\rm applied}-\tau_{\rm constraint}.$$
The norm of $r$ should be near numerical precision after a consistent forward calculation. This check catches stale data and missing damping forces. It does not prove the physical model matches a real robot. Reconstruct $M$ with the provided API, not by reshaping its internal sparse storage.

A free joint stores seven configuration numbers and six velocity numbers. A state derivative or controller gain built with nq where nv belongs may have the wrong meaning even when the array operations happen to succeed.

## Differentiation and batching

The first labs use CPU MuJoCo. Later, MJX allows batches of compatible models on accelerators. Its JAX path supports functional data updates and JAX transformations. Backend support and contact differentiability need separate checks [@mujoco-mjx]. Report compilation time separately from steady-state execution.

## Exercise and answer check

Halve the physics timestep while keeping the controller at 100 Hz. Which parameter changes? **Answer:** double the substep count. Compare trajectories at the same physical times. If instead you keep the substep count fixed, you have changed both the numerical approximation and the controller rate, so the experiment cannot isolate integration error.
