# Computational Analysis Monitor

This package now includes a lightweight online monitor for computational-cost experiments:

- script: `scripts/computational_analysis.py`
- install target: `lib/adaptive_odom_filter/computational_analysis.py`

The monitor is intended to run while the full ROS 2 pipeline is active. It focuses on the odometry-fusion filter, but it can also track other processes and ROS 2 topics so you can compare the filter against FAST-LIO2, AMCL, or any additional node in the experiment.

## What it measures

The script collects three groups of metrics.

### 1. Process metrics

Sampled at a configurable low rate with `psutil`.

- CPU usage normalized to the full machine: suitable for tables like `CPU Odom. (%)`
- CPU usage relative to a single core: useful for debugging saturation
- resident memory (RSS) in MB
- total thread count
- context-switch rate
- read/write throughput when available from the OS

Process monitoring is regex-based, which lets you aggregate multiple PIDs under one logical label such as `filter`, `fastlio`, or `amcl`.

### 2. Topic timing metrics

Collected from lightweight ROS 2 subscriptions.

- message count
- effective output rate over the session
- inter-arrival statistics: mean, standard deviation, 95th percentile, and worst gap
- message age from `header.stamp` to reception time when the message carries a header

These topic metrics are especially useful for:

- verifying the effective frequency of `/ekf_loam/filter_odom_to_init`
- checking whether the filter output rate actually follows the intended runtime
- quantifying input/output timing quality for wheel odometry, IMU, LiDAR odometry, and AMCL

### 3. System-level metrics

- total CPU usage
- RAM usage percentage
- RAM used in MB

These values help interpret process-level measurements in the context of the full machine load.

## Output files

Each run creates a timestamped directory under `analysis_out/`, unless `--output-dir` is specified.

Files written by default:

- `process_samples.csv`
  One row per sample period and per monitored process label.
- `topic_samples.csv`
  One row per sample period and per monitored topic label.
- `system_samples.csv`
  One row per sample period for machine-level load.
- `process_summary.csv`
  End-of-run summary for each monitored process label.
- `topic_summary.csv`
  End-of-run summary for each monitored topic label.
- `system_summary.csv`
  End-of-run summary for machine-level load.
- `session_summary.json`
  Run metadata, monitor configuration, and output paths.

The raw sample CSVs are useful for later plotting. The summary CSVs are the fastest path to a revised computational-cost table in the paper.

## Default monitored items

Unless disabled, the script starts with these defaults.

### Default process labels

- `filter=adaptive_odom_filter|EKFAdaptiveFilter|EKFRobustAdaptiveFilter`
- `fastlio=fastlio|fast_lio|ekf_loam`
- `amcl=(^|/|\\s)amcl($|\\s)`

### Default topic labels

- `wheel_odom=/odom_with_cov`
- `imu=/imu/data`
- `lidar_odom=/ekf_loam/laser_odom_with_cov`
- `filter_odom=/ekf_loam/filter_odom_to_init`
- `amcl_pose=/amcl_pose`

If your actual process names or topic names differ, keep the defaults and add explicit overrides, or disable the defaults and provide your own labels.

## Typical usage

### Recommended online run

Run this in a separate terminal while the full pipeline is active:

```bash
python3 scripts/computational_analysis.py \
  --duration 180 \
  --sample-period 1.0 \
  --summary-period 5.0
```

This records a 3-minute window with a low-overhead 1 Hz system/process sampling period and concise 5-second summaries in the terminal.

### Focus on the filter with explicit overrides

If the process or topic names differ in your experiment:

```bash
python3 scripts/computational_analysis.py \
  --duration 180 \
  --process filter=EKFAdaptiveFilter \
  --process fastlio=lio|fastlio|ekf_loam \
  --process amcl=amcl \
  --topic filter_odom=/filter_odom \
  --topic lidar_odom=/lidar_odom
```

### Use only custom labels

```bash
python3 scripts/computational_analysis.py \
  --no-default-processes \
  --no-default-topics \
  --process filter=EKFAdaptiveFilter \
  --process amcl=amcl \
  --topic filter_odom=/filter_odom \
  --topic amcl_pose=/amcl_pose
```

### Lightweight smoke test

```bash
python3 scripts/computational_analysis.py \
  --duration 2 \
  --sample-period 0.5 \
  --summary-period 1 \
  --monitor-self \
  --no-default-processes \
  --no-default-topics
```

This is useful when validating the script itself on a machine without a live pipeline.

## How to read the summaries

### `process_summary.csv`

Most relevant fields for the paper:

- `mean_cpu_percent_system`
- `p95_cpu_percent_system`
- `max_cpu_percent_system`
- `mean_rss_mb`
- `p95_rss_mb`
- `max_rss_mb`
- `mean_threads`

Suggested use:

- report the mean CPU and memory in the paper table
- use the 95th percentile values internally to judge stability during longer runs

### `topic_summary.csv`

Most relevant fields:

- `effective_rate_hz`
- `mean_inter_arrival_ms`
- `std_inter_arrival_ms`
- `p95_inter_arrival_ms`
- `max_gap_ms`
- `mean_age_ms`
- `p95_age_ms`

Suggested use:

- use `effective_rate_hz` to report the actual output frequency of the filter
- use inter-arrival statistics to quantify jitter
- use age statistics to discuss how fresh the output is when received online

## Recommended experimental workflow

For each online run:

1. Start the normal pipeline.
2. Start this monitor in a second terminal.
3. Let the robot execute the full test trajectory.
4. Stop the monitor after the trajectory ends, or use `--duration`.
5. Archive the generated directory together with the bag and the test identifier.

If you want one summary directory per experiment condition, pass a condition-specific `--output-dir`.

Examples:

```bash
python3 scripts/computational_analysis.py \
  --duration 240 \
  --output-dir /tmp/filter_cost_square_run01
```

## Overhead considerations

The monitor was written to avoid becoming a significant load source during the experiment.

- Process and system polling are low-rate and use `psutil`.
- Topic callbacks store only timing metadata, not full message contents.
- Console output is periodic and compact.
- CSV writing is incremental, so the monitor does not need to hold raw sample history in RAM before saving it.

If you want even less overhead:

- increase `--sample-period` from `1.0` to `2.0`
- increase `--summary-period`
- disable topic labels you do not need

## Notes on interpretation

- CPU usage is reported both per core and normalized to the full machine. For paper tables, prefer the normalized full-machine values.
- Message age depends on valid `header.stamp` fields and a consistent time base.
- If a process label stays at zero PIDs, the regex did not match the running process name or command line. In that case, add a more specific `--process` rule.
- If a topic label remains unsubscribed, verify the topic name and the message type actually published by the pipeline.
