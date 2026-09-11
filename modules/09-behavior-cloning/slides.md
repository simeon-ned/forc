## Behavior cloning

$$\min_\theta \frac1N\sum_i\|\pi_\theta(o_i)-u_i^{\rm expert}\|^2$$

Training uses demonstrations.

Execution uses the student's own observations and actions.

---
## Dataset contract

Complete episodes retain reset conditions and termination reasons.

Training and evaluation use different episodes.

Normalization uses training data only.

---
## Distribution shift

A small action error changes the next state.

The student can enter regions absent from demonstrations.

---
## Linear sanity check

A linear student can recover a linear expert from sufficiently rich noiseless data.

This validates the pipeline before adding complex models.

---
## Evaluation

Action MSE measures prediction on a dataset.

Rollout cost measures closed-loop consequences.

Both belong in the report.

---
## Practice

Fit a small student, then test broader initial states.

Which failures appear only in rollouts?

Reading: [@mit-manipulation]
