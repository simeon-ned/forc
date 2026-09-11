## Dataset aggregation

The learner visits states.

The expert labels those states.

Retraining expands the supervised dataset.

---
## Acting and labeling

The acting policy controls the trajectory distribution.

The labeling policy supplies the target actions.

These roles can belong to different policies.

---
## Training loop

Fit, collect learner rollouts, query expert labels, aggregate, retrain.

Final policy selection uses held-out rollouts.

---
## Expert-query budget

A slow optimizer or human expert has a real labeling cost.

Report queries as well as environment steps.

---
## Limits

Unrecoverable states remain unrecoverable.

A restricted policy class may retain imitation error.

---
## Check

The learner labels new observations with its own actions.

What ingredient of DAgger is missing?
