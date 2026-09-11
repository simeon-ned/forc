## Learning an input rule from demonstrations

Behavior cloning fits a policy to expert state-action pairs:
$$\min_\theta \frac1N\sum_i\|\pi_\theta(o_i)-u_i^{\rm expert}\|^2.$$
The observation $o_i$ must contain only information available when the learned policy runs. If an expert uses exact simulator state but a student sees noisy sensors, imitation may be intrinsically ambiguous. Include observation normalization fitted on training data only.

The first baseline should be deliberately simple. A linear policy on a low-dimensional system reveals the data pipeline without conflating representation learning, optimization, and control. More expressive policies are justified when the expert action map cannot be represented adequately.

## Demonstration collection

Run the expert from diverse initial conditions with a fixed controller rate and actuator limits. Save complete episodes with their initial state, observations, actions, and termination reason. Split by episode and initial-condition regime. A random row split leaks almost identical neighboring states between training and test sets.

Fit on expert actions after the same clipping used during execution. If you instead train on unreachable requested actions, the optimization target and deployed controller differ.

## Worked distribution-shift example

Imagine an expert balancing a system near zero. Its dataset contains almost no states with a large angle. A small prediction error moves the student outside that region; the next observation is less familiar, producing another error. A low average action error on held-out expert trajectories can coexist with rapid closed-loop failure.

For a linear expert $u=-Kx$ and a linear student, sufficiently rich noiseless training states can recover the expert exactly. This is a useful pipeline test. It does not demonstrate that behavior cloning is immune to distribution shift on nonlinear tasks. Our practice deliberately limits the student feature set and training coverage so the evaluation protocol remains important.

## Evaluation before more training

Report action prediction error on held-out demonstrations separately from rollout return, success rate, constraint violations, and expert-query count. Evaluate nominal and perturbed starts. Repeat training with several seeds when the fitting procedure or dataset is stochastic.

A learned policy trained in simulation may exploit unrealistic dynamics or observations. Testing mass variation and noise is useful evidence, but not proof of transfer to hardware.

## Practice and exercise

Practice 5 trains a small polynomial policy to imitate a saturated feedback expert. It then compares expert-distribution test error with closed-loop costs on broader initial conditions. The next lecture adds expert labels on student-visited states.

**Question:** Should a policy with lower validation MSE always have lower rollout cost? **Answer:** no. The errors may occur at different states, and some states are far more consequential to the future trajectory. Closed-loop evaluation measures the effects that the supervised objective only approximates.

The manipulation course provides context for imitation in richer robotic settings [@mit-manipulation]. The present practice isolates the control and dataset issues before introducing image encoders or large networks.
