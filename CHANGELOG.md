# Changelog

## 2026-04-20

### Computational analysis tooling

Files changed:

- `CMakeLists.txt`
- `package.xml`
- `scripts/computational_analysis.py`
- `scripts/computational_analysis.md`
- `CHANGELOG.md`

Summary:

- added a lightweight online computational monitor for the ROS 2 odometry pipeline
- installed the new script as a package program under `lib/adaptive_odom_filter`
- added runtime dependencies for `psutil` and `rosidl_runtime_py`
- documented the tool in detail, including outputs, defaults, interpretation, and usage patterns

What the new script does:

- samples per-process CPU, memory, thread count, context-switch rate, and I/O throughput with low overhead
- subscribes to the main ROS 2 topics and measures effective rate, inter-arrival jitter, maximum gaps, and message age from `header.stamp`
- writes both raw sample CSVs and end-of-run summary CSVs so experimental sessions can be compared directly afterward
- prints compact periodic summaries during the run instead of flooding the terminal

Default monitored process labels:

- `filter=adaptive_odom_filter|EKFAdaptiveFilter|EKFRobustAdaptiveFilter`
- `fastlio=fastlio|fast_lio|ekf_loam`
- `amcl=(^|/|\\s)amcl($|\\s)`

Default monitored topic labels:

- `wheel_odom=/odom_with_cov`
- `imu=/imu/data`
- `lidar_odom=/ekf_loam/laser_odom_with_cov`
- `filter_odom=/ekf_loam/filter_odom_to_init`
- `amcl_pose=/amcl_pose`

Main outputs written per run:

- `process_samples.csv`
- `system_samples.csv`
- `topic_samples.csv`
- `process_summary.csv`
- `system_summary.csv`
- `topic_summary.csv`
- `session_summary.json`

Typical usage:

- `python3 scripts/computational_analysis.py --duration 180 --sample-period 1.0 --summary-period 5.0`
- use `--process label=regex` and `--topic label=/topic_name` to adapt the monitor to the exact runtime names used in each experiment
- use `--output-dir` to tie each results folder to a single experimental condition or run identifier

Validation performed on this branch:

- `python3 -m py_compile scripts/computational_analysis.py`
- `python3 scripts/computational_analysis.py --duration 2 --sample-period 0.5 --summary-period 1 --monitor-self --no-default-processes --no-default-topics --output-dir /tmp/adaptive_odom_filter_monitor_smoke`
- started `python3 scripts/fake_odometry_path.py`
- `python3 scripts/computational_analysis.py --duration 3 --sample-period 0.5 --summary-period 1 --monitor-self --process fake_publisher=fake_odometry_path.py --output-dir /tmp/adaptive_odom_filter_monitor_live`

Observed behavior during validation:

- the monitor produced the expected CSV and JSON outputs
- topic discovery succeeded for the fake wheel odometry, LiDAR odometry, and IMU streams
- live summaries reported process CPU/RSS plus online topic-rate and message-age statistics
- the monitor shut down cleanly after the live test
	
## 2026-04-18

### Commit `9470d1e`
Port latest master filter updates into the ROS 2 package.

Files changed:

- `.gitignore`
- `CMakeLists.txt`
- `package.xml`
- `config/adaptive_filter_parameters.yaml`
- `config/adaptive_robust_filter_parameters.yaml`
- `include/settings_adaptive_filter.h`
- `include/adaptive_odom_filter/ekf_adaptive_tools.h`
- `include/adaptive_odom_filter/adaptive_robust_ekf.h`
- `include/adaptive_odom_filter/alphaBetaFilter.h`
- `launch/adaptive_filter_fastlio_wheelodom.launch.py`
- `launch/adaptive_filter_odom.launch.py`
- `launch/adaptive_robust_filter_ekf_loam.launch.py`
- `launch/adaptive_robust_filter_odom.launch.py`
- `scripts/fake_odometry_path.py`
- `src/EKFAdaptiveFilter.cpp`
- `src/EKFRobustAdaptiveFilter.cpp`
- `src/ekf_adaptive_tools.cpp`
- `src/adaptive_robust_ekf.cpp`
- `src/adaptive_robust_ekf_first.cpp`
- `src/alphaBetaFilter.cpp`

Summary:

- ported the latest adaptive filter logic from `master` into the ROS 2 package
- added the robust EKF core and ROS 2 robust node
- added the shared alpha-beta filter utility used by the updated filter logic
- updated adaptive EKF handling for wheel measurement layout, lidar covariance flow, timing checks, and velocity smoothing
- restructured the package build around a shared library and multiple ROS 2 executables
- added ROS 2 launch files and parameter files for adaptive and robust pipelines
- aligned the runtime shell with ROS 2 conventions while keeping the filter math and behavior close to the newer `master` implementation

### Commit `251d771`
Drop generated colcon artifacts from the ROS 2 package.

Files changed:

- tracked files under `build/`
- tracked files under `install/`
- tracked files under `log/`

Summary:

- removed generated colcon outputs from version control
- made the branch safe for fresh clones in ROS 2 workspaces
- kept generated files local to each developer workspace, which is the expected ament/colcon workflow

Validation done after cleanup:

- `colcon --log-base /tmp/adaptive_odom_filter_colcon_log build --symlink-install --build-base /tmp/adaptive_odom_filter_colcon_build --install-base /tmp/adaptive_odom_filter_colcon_install`
- build completed successfully

### Working tree documentation update

Files changed:

- `README.md`
- `CHANGELOG.md`

Summary:

- rewrote the README for the current ROS 2 package layout and usage
- documented build flow, launch flow, main topics, major tuning parameters, and common runtime issues
- added this changelog file to record branch evolution in a simple technical format
