# Changelog

## 2026-05-12

### ROS 2 adaptive EKF correction and bag-test tuning

Files changed:

- `include/adaptive_odom_filter/ekf_adaptive_tools.h`
- `src/ekf_adaptive_tools.cpp`
- `src/EKFAdaptiveFilter.cpp`
- `config/adaptive_filter_parameters.yaml`
- `launch/adaptive_filter_fastlio_wheelodom.launch.py`
- `CHANGELOG.md`

What changed:

- I made the indirect odometry measurement model pure, so the numerical Jacobian code no longer mutates the alpha-beta filter state while it probes the model.
- I kept alpha-beta smoothing in the LiDAR and visual correction path, but I now apply it exactly once for each real correction.
- I made the LiDAR correction distinguish pose covariance from twist covariance. When Fast-LIO2 publishes pose covariance, the filter now propagates it through the pose-delta-to-body-velocity Jacobians; when the node falls back to twist covariance, the filter uses it directly.
- I added covariance-mode bookkeeping for the current and previous LiDAR samples, so the propagated covariance uses a consistent interpretation for both sides of the finite-difference velocity estimate.
- I added a corrected-state sequence number and last-update source to the EKF worker thread.
- I moved filtered odometry publication out of the LiDAR callback and into a timer, so the ROS 2 node publishes only states that the filter thread has already predicted and corrected.
- I protected output headers with a mutex, which keeps the timer publisher and sensor callbacks from racing on the selected header.
- I changed `get_state()` to return the cached output state instead of reading the mutable internal EKF state directly.
- I replaced the hard-coded wheel velocity conversion `-0.705 * /odom.twist.twist.linear.x` with the `wheelVxScale` parameter.
- I set `wheelVxScale: 1.0`, so the default wheel `vx` path no longer flips or shrinks `/odom.twist.twist.linear.x`.
- I relaxed `wheelGVx` from `0.001` to `0.1`, so wheel longitudinal velocity no longer dominates the LiDAR-derived velocity as aggressively after the LiDAR covariance fix.
- I set `use_sim_time` to `true` in `adaptive_filter_fastlio_wheelodom.launch.py` for bag playback tests.

Why it changed:

- The previous Jacobian path updated the alpha-beta filter during finite-difference probes, which could corrupt the correction input before the EKF used it.
- The previous LiDAR covariance path treated pose covariance too much like velocity covariance, which made LiDAR confidence inconsistent with the measurement model.
- The previous callback publication path could publish a state before the worker thread finished the correction cycle.
- The previous wheel `vx` sign and scale made the filtered position look mirrored while orientation still looked correct when `/odom` already used the expected forward convention.
- The previous `wheelGVx` value trusted wheel forward velocity strongly enough to shrink the filtered lemniscate when wheel scale/sign disagreed with Fast-LIO2 or ground truth.

Validation and test context:

- I ran static source review and `git diff --check`.
- I did not build or launch the package locally during the implementation phase because the test plan requested code-only changes.
- Diogo tested the node with ROS 2 bag playback, including `ros2 launch adaptive_odom_filter adaptive_filter_fastlio_wheelodom.launch.py use_sim_time:=true`, and confirmed that the corrected path behaved well on bag data.
- Diogo also tested the wheel sign/scale tuning with bag playback and reported that the mirrored position and smaller lemniscate behavior looked corrected.

What to watch next:

- Check `wheelVxScale` against each robot or bag source before using the filter on a different odometry convention.
- Re-evaluate `wheelGVx`, `lidarG`, and the effective LiDAR velocity covariance on bag data before the next robot run.
- Compare `/odom`, `/Odometry`, `/filter_odom`, and ground truth path scale in RViz after every gain change.

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
