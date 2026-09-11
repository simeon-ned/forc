## Aim

Implement and inspect reward-based learning before moving to neural robot policies.

**Time:** about 120 minutes. Start with the linked lectures and the course notation contract. The reference program is complete and runnable; your work is to explain it, modify it, and test the results.

## Run from this repository

~~~bash
uv sync --locked
uv run --locked python practices/p06-reinforcement-learning/run.py
~~~

The program writes configuration, MuJoCo version, and metrics to outputs/p06-reinforcement-learning/metrics.json. Experiments with a time history also write trajectory.csv. These are generated results and are not source material.

Use a separate configuration and output directory for each variation:

~~~bash
uv run --locked python practices/p06-reinforcement-learning/run.py --config practices/p06-reinforcement-learning/config.yml --output outputs/p06-reinforcement-learning/baseline
~~~

Keep the original configuration as the baseline. Record all changes in your report; avoid changing several physical and numerical parameters at once.

## Tasks

1. Write out the 41-state position-regulation MDP and three actions. The dt field labels the experiment clock; transitions are the specified discrete grid moves.
2. Verify the Bellman arithmetic from Lecture 11, including the terminal case.
3. Compare the frozen learned policy with random and hand-designed baselines from every nonterminal state.
4. Sweep exploration and discount. Keep training cutoffs separate from true goal termination.

## Expected checks

Evaluation disables exploration. The hand-designed policy moves toward the center and supplies a transparent reference. This exercise does not claim to train a continuous-control or locomotion policy.

## Submission

Submit your YAML configuration, a short explanation of the equations and assumptions, labeled plots or numerical comparisons with units, and a paragraph explaining a failure or limitation. Include the exact run command. The weekly practice rubric awards 40% for correctness and checks, 30% for the controlled comparison, 20% for interpretation, and 10% for reproducibility.

## Extension

Replace the finite-state task with a continuous pendulum and document what changes in observation, action, approximation, and evaluation. Treat that as a new experiment.

## Reading

[@cmu-control] · [@mit-underactuated]
