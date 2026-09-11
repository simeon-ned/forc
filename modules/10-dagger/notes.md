## Correcting the training distribution

Behavior cloning learns from states visited by the expert. Dataset Aggregation, or DAgger, asks the expert what to do at states visited by the learner. Start from an initial demonstration dataset, train a policy, roll it out, query expert actions at its visited observations or states, aggregate those new labels, and retrain.

The distinction between acting and labeling is essential. If the expert acts for the entire rollout, the dataset remains concentrated on expert trajectories. A mixture policy can use the expert with probability $\beta_i$ during early iterations, but the student must eventually generate its own state distribution.

## A concrete training loop

1. Fit the student to the accumulated dataset.
2. Reset the environment to a declared training initial-state distribution.
3. Select student or expert action according to the current mixture schedule.
4. Record the current observation and the expert's label at that state.
5. Add the labeled episode to the dataset.
6. Select the final policy using held-out rollout evaluation.

The practice implements the pure-student rollout variant after initialization. The expert is a simulation controller, so labels are inexpensive. On hardware, an expert may be a human or a slow optimization procedure, and queries may be costly or unsafe.

## Worked boundary case

Suppose the student reaches a state from which the actuator cannot prevent failure. The expert label can still be well-defined, but no algorithm can turn that state into a successful recovery if the physics forbids it. Aggregation may help the student avoid entering it in future episodes. It cannot guarantee recovery at every labeled state.

Likewise, if the student's feature class cannot represent the expert, collecting more labels may not eliminate error. Our small polynomial student exposes this issue. Inspect whether additional rounds reduce rollout cost, merely reduce training loss, or change neither.

## Fair comparisons

Compare BC and DAgger under a declared expert-label budget. Report both the number of episodes and the number of queries. Evaluate every saved policy on the same independent initial states and disturbances. Do not tune the mixture schedule on the final test set.

Aggregation changes which states dominate the loss. A large old dataset can dilute recent failure-state labels; aggressive reweighting can forget nominal behavior. Either design is an additional algorithm choice, and should be recorded.

## Exercise and answer guide

A student collects 1,000 new observations but stores its own actions as labels. Has it implemented DAgger? **No.** It has reinforced its current decisions rather than obtained corrective expert labels.

Next ask why a DAgger curve might get worse after an iteration. Possible explanations include finite data, optimization error, a mismatched student class, inconsistent expert labels, or evaluation noise. The algorithmic idea motivates collecting better-distributed supervision; it is not a promise of monotonically improving every finite experiment.

Read the learning sections of the manipulation notes for broader policy-learning context [@mit-manipulation].
