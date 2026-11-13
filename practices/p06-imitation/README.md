## Aim

Separate supervised prediction quality from closed-loop behavior under distribution shift.

**Time:** about 120 minutes. Start with the linked lectures and the course notation contract. The reference program is complete and runnable; your work is to explain it, modify it, and test the results.

## Run from this repository

Run from the repository root. See the [shared setup guide](../README.md) for uv, conda, rendering, and installing all practices.

~~~bash
uv sync --locked --extra p06
uv run --locked --extra p06 python practices/p06-imitation/run.py
~~~

With the course conda environment activated:

~~~bash
python scripts/setup_conda.py p06
python practices/p06-imitation/run.py
~~~

The program writes configuration, available MuJoCo version (null when not installed), and metrics to outputs/p06-imitation/metrics.json. Experiments with a time history also write trajectory.csv. These are generated results and are not source material.

Use a separate configuration and output directory for each variation:

~~~bash
uv run --locked --extra p06 python practices/p06-imitation/run.py --config practices/p06-imitation/config.yml --output outputs/p06-imitation/baseline
~~~

Keep the original configuration as the baseline. Record all changes in your report; avoid changing several physical and numerical parameters at once.

## Tasks

1. Inspect the expert controller, student features, episode split, and evaluation initial states.
2. Run BC as round zero, then inspect each dataset-aggregation round.
3. Plot expert-distribution validation MSE against broader-distribution rollout cost.
4. Repeat with three seeds and a second regularization value. Report expert queries and any worsening iteration.

## Expected checks

DAgger labels learner-visited states with expert actions. Evaluation states remain fixed across rounds. Improvement is an empirical result to measure, not a guaranteed property of this restricted polynomial student.

## Submission

Submit your YAML configuration, a short explanation of the equations and assumptions, labeled plots or numerical comparisons with units, and a paragraph explaining a failure or limitation. Include the exact run command. The weekly practice rubric awards 40% for correctness and checks, 30% for the controlled comparison, 20% for interpretation, and 10% for reproducibility.

## Extension

Compare BC and DAgger under a matched label budget. Add observation noise without giving the deployed student privileged state.

## Reading

[@mit-manipulation] · [@cmu-control]
