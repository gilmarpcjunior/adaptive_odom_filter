# Offline Bayesian optimization for the adaptive odometry filter

This folder contains experiment tooling for tuning the ROS 2 adaptive EKF by
running the real filter node on recorded bags, recording a clean live filter
topic, and scoring the output against a ground-truth path.

The optimizer intentionally treats the filter as a black box. It does not tune a
simplified MATLAB or Python surrogate model. Each candidate launches
`adaptive_odom_filter_node`, replays one or more bags with `/clock`, records a
candidate-specific output topic, evaluates the resulting path, and then updates a
Gaussian-process expected-improvement optimizer.

## Objective

The ground-truth topic in the current bags has timestamp/frequency issues, so
the objective compares path geometry rather than trajectory timing:

- resample each odometry path by normalized arc length
- compute path-only `x`, `y`, `z`, XY, XYZ, and yaw errors
- penalize path-length scale error
- penalize adjacent output jumps
- compare the filter against FAST-LIO2 on the same bag

The aggregate optimization score is the mean bag score plus a small stability
penalty for bag-to-bag variance. Lower is better.

## Typical usage

From the repository root:

```bash
python3 optimization/bayesian_filter_optimizer.py analyze \
  --bag /home/diogo/bag3_odom_ros2dev_010526 \
  --bag /home/diogo/bag4_odom_ros2dev_010526 \
  --bag /home/diogo/bag5_odom_ros2dev_010526 \
  --bag /home/diogo/bag6_odom_ros2dev_010526 \
  --output-dir /tmp/adaptive_odom_filter_bayes_analysis
```

```bash
python3 optimization/bayesian_filter_optimizer.py optimize \
  --train-bag /home/diogo/bag4_odom_ros2dev_010526 \
  --train-bag /home/diogo/bag5_odom_ros2dev_010526 \
  --train-bag /home/diogo/bag6_odom_ros2dev_010526 \
  --validation-bag /home/diogo/bag3_odom_ros2dev_010526 \
  --wheel-mode adaptive \
  --max-evals 12 \
  --init-evals 5 \
  --play-rate 1.0 \
  --startup-wait 5.0 \
  --output-dir /tmp/adaptive_odom_filter_bayes_run
```

The script writes CSV, JSON, Markdown summaries, convergence plots, and per-bag
path/error plots for the best candidate.

Wheel modes:

- `--wheel-mode fixed` tunes the fixed `wheelGVx`, `wheelGVy`, and `wheelGWz`
  covariance path with `wheel_type_func=1`.
- `--wheel-mode adaptive` tunes `wheel_type_func=0`, including `wheelOffset`,
  `gamma_vx`, `gamma_omegaz`, `delta_vx`, and `delta_omegaz`.
- `--wheel-mode none` disables wheel correction and tunes only LiDAR-derived
  velocity confidence. This is the baseline every wheel-fused candidate should
  beat.

Use `--play-rate 1.0` for final numbers. Faster replay can under-sample the
timer-published filter output and bias the optimizer toward candidates that only
look good under replay stress.

Use a longer `--startup-wait` for final scoring. Short startup windows can make
the first filter messages sensitive to ROS discovery and timer scheduling. If a
candidate is close to being selected, repeat it and use the median score.
