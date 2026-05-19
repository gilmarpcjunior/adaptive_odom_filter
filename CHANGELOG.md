# Changelog

## 2026-05-19

### ROS 2 development branch stabilization and offline gain analysis

Files changed:

- `CMakeLists.txt`
- `config/adaptive_filter_parameters.yaml`
- `include/adaptive_odom_filter/ekf_adaptive_tools.h`
- `launch/adaptive_filter_fastlio_wheelodom.launch.py`
- `optimization/README.md`
- `optimization/bayesian_filter_optimizer.py`
- `src/EKFAdaptiveFilter.cpp`
- `src/ekf_adaptive_tools.cpp`
- `CHANGELOG.md`

What changed:

- The EKF correction covariance update now uses the same measurement covariance used to compute each Kalman gain.
- The LiDAR correction path now uses the raw finite-difference odometry velocity instead of applying the alpha-beta velocity smoother in the main LiDAR update.
- The filter output cache now honors `filterFreq`, so the default `filterFreq: "l"` publishes only LiDAR-corrected output states.
- The launch file now exposes `use_sim_time`, `use_imu`, and `use_wheel` as launch arguments.
- The runtime parameter profile now disables IMU and wheel correction by default, keeps LiDAR enabled, uses `lidarG=0.5`, `alpha_lidar=1.0`, and keeps `wheel_type_func=0` ready for adaptive-wheel tests.
- The offline optimizer now supports real filter-in-the-loop Bayesian evaluation, LiDAR-only evaluation, adaptive-wheel parameters, and per-bag diagnostic plots.

Why it changed:

- The old Joseph covariance update made the posterior covariance overconfident because it used `_epsR * I` instead of the real measurement covariance.
- The alpha-beta smoothing path could shorten and distort the filtered path when FAST-LIO2 already provided the dominant odometry source.
- The old publisher could expose intermediate states from IMU or wheel corrections even when the desired output source was LiDAR.
- The recorded wheel and IMU covariances did not provide enough reliable independent information to improve the filter consistently on the tested bags.

Validation and test context:

- The package built successfully with `colcon build --packages-select adaptive_odom_filter --symlink-install`.
- The current `ros2-dev` LiDAR-only profile was evaluated on `bag3_odom_ros2dev_010526`, `bag4_odom_ros2dev_010526`, `bag5_odom_ros2dev_010526`, and `bag6_odom_ros2dev_010526` with `use_sim_time=true` and bag playback.
- The current LiDAR-only profile reached mean path-only XY RMSE `0.2155 m` against `/scout_mini/Odom`, while FAST-LIO2 reached `0.2268 m` on the same metric, and the filter beat FAST-LIO2 on two of four bags.
- The adaptive-wheel candidate reached mean XY RMSE `0.3087 m`, so wheel fusion still degraded the aggregate result on the recorded bags.
- An isolated `ros2` branch worktree with the known fail-recovery patch reached mean XY RMSE `0.4210 m`, so the current `ros2-dev` profile remained better than the `ros2` branch test.
- FAST-LIO2-only loop-bag tests confirmed that the filter cannot create true loop closure without an independent correction source such as reliable wheel/IMU covariance, AMCL/map correction, or explicit pose correction.

What to watch next:

- Test the committed `ros2-dev` profile online before changing gains again.
- Record truthful wheel, IMU, and LiDAR covariances during robot tests; the optimizer should run again after those covariances exist.
- Treat any FAST-LIO2-only loop-closure improvement as smoothing or timing behavior, not as real loop closure.

## 2026-05-15

### EKF correction covariance consistency

Files changed:

- `launch/adaptive_filter_fastlio_wheelodom.launch.py`
- `src/ekf_adaptive_tools.cpp`
- `CHANGELOG.md`

What changed:

- The Joseph covariance update in the wheel, IMU, LiDAR, and visual correction stages now uses the same measurement covariance used by each Kalman gain.
- Wheel corrections now update the posterior covariance with `E_wheel`, IMU corrections use the IMU orientation covariance block, and LiDAR/visual corrections use the propagated odometry covariance `Q`.
- `adaptive_filter_fastlio_wheelodom.launch.py` now exposes `use_sim_time`, `use_imu`, and `use_wheel` launch arguments, with `use_imu:=false` by default and `use_wheel:=true` by default.

Why it changed:

- The previous correction code computed the Kalman gain with the sensor covariance, then updated `_P` with `_epsR * I`, which made the posterior covariance artificially small after every correction.
- That overconfidence can let a wheel correction keep influencing the state after the wheel model becomes wrong, especially during turns, and it can prevent later FAST-LIO2 updates from pulling the estimate back with the correct authority.
- The current bag analysis shows the wheel forward velocity agrees reasonably with FAST-LIO2-derived velocity, but wheel yaw rate disagrees more during turns; the filter therefore needs honest posterior covariance propagation before gain tuning can mean anything.
- The recorded bags publish zero IMU covariance, so enabling IMU orientation correction by default can make the filter trust the IMU attitude as nearly perfect.

What to watch next:

- Rebuild before testing, because this change is in C++ filter code.
- Re-run the optimizer after this correction; previous gain results came from the old overconfident covariance update and should serve only as references.
- Keep IMU orientation correction disabled unless the robot publishes meaningful IMU orientation covariance or the code adds covariance floors; enable it explicitly with `use_imu:=true` only for that test.

## 2026-05-13

### Offline Bayesian optimization tooling

Files changed:

- `optimization/README.md`
- `optimization/bayesian_filter_optimizer.py`
- `CMakeLists.txt`
- `CHANGELOG.md`

What changed:

- A separate offline optimizer now runs the real ROS 2 adaptive EKF on recorded bags instead of tuning a simplified surrogate model.
- The optimizer launches `adaptive_odom_filter_node`, replays each bag with `/clock`, records a unique live filter output topic, and ignores old bagged `/filter_odom` data.
- The optimizer implements a Gaussian-process expected-improvement loop with `numpy` and `scipy`-free math, because this machine does not currently have `sklearn` installed.
- The objective compares path geometry by normalized arc length instead of timestamp-aligned trajectory samples.
- The score combines XY path RMSE against `/scout_mini/Odom`, path-length scale error, yaw path error, adjacent jump penalties, and a penalty when the filter does not beat FAST-LIO2.
- The tool writes baseline metrics, per-candidate metrics, convergence plots, four-panel path plots, overlay plots, and `x`, `y`, `z`, and yaw error plots for the best candidate.
- The final-replay default stays at `--play-rate 1.0`, because a 3x smoke replay under-sampled the timer-published filter output and distorted the path length.
- The optimizer/evaluator default startup wait is now `5.0 s` after repeated bag3 tests showed that shorter startup timing can change the scored path.

Why it changed:

- The ground-truth topic in the bags has timestamp/frequency issues, so strict time alignment would optimize against a timing artifact instead of the geometric path.
- The filter behavior depends on callback timing, output gating, ROS parameters, message rates, and the real covariance handling path, so the optimizer needs to evaluate the actual node.
- Candidate output topics need unique names because the input bags already contain stale `/filter_odom` messages.

What to watch next:

- Use several bags for training and keep at least one bag as validation, otherwise the selected gains can overfit one path.
- Re-run the final best candidate at `--play-rate 1.0` before copying gains into the runtime parameter file.
- Repeat close candidates and compare median scores, because live ROS replay plus timer-published output introduces measurable run-to-run variation.
- Treat `enableImu` as a candidate parameter until the IMU zero-covariance issue gets a code-level covariance floor or a dedicated orientation-use parameter.

Experiment results from the first run:

- Baseline path-only analysis ran on `bag3_odom_ros2dev_010526`, `bag4_odom_ros2dev_010526`, `bag5_odom_ros2dev_010526`, and `bag6_odom_ros2dev_010526`.
- A normal-speed Bayesian sweep ran on bags 4, 5, and 6, then the training-best candidate was validated on bag3.
- The training-best candidate used `enableImu=false`, `lidarG=0.21424023`, `wheelGVx=0.68471723`, `wheelGVy=2.7036176`, `wheelGWz=66.342402`, `alpha_lidar=0.9986638`, and `wheelVxScale=1.06801`.
- That training-best candidate improved the training mean XY RMSE to `0.2577 m`, but it failed validation on bag3 with `0.8246 m` XY RMSE against FAST-LIO2 at `0.3841 m`.
- A more robust hand/BO anchor used `enableImu=false`, `lidarG=0.75`, `wheelGVx=1.0`, `wheelGVy=5.0`, `wheelGWz=80.0`, `alpha_lidar=0.995`, and `wheelVxScale=1.0`.
- That robust candidate beat FAST-LIO2 on bag6 in repeated runs and improved bags 4 and 5 compared with the current filter, but it did not consistently beat FAST-LIO2 across all four bags.
- Repeated bag3 runs for the robust candidate produced `0.2776 m`, `0.6059 m`, and `0.3925 m` XY RMSE, so any single replay should not serve as final gain proof.

### Adaptive wheel and LiDAR-only optimization follow-up

Files changed:

- `optimization/bayesian_filter_optimizer.py`
- `config/adaptive_filter_parameters.yaml`
- `CHANGELOG.md`

What changed:

- The optimizer can now run `wheel_type_func=0` and tune `wheelOffset`, `gamma_vx`, `gamma_omegaz`, `delta_vx`, and `delta_omegaz`.
- The optimizer now includes an explicit LiDAR-only mode with `enableWheel=false`, because the adaptive-wheel tests showed that wheel corrections still pull the filter away from the best path on several bags.
- Optimizer candidates and explicit candidate evaluation now include `enableWheel`, so the optimizer can compare wheel-fused and LiDAR-only filter behavior.
- The active parameter file moved toward the best LiDAR-only test profile: `enableWheel=false`, `enableImu=false`, `lidarG=0.5`, and `wheel_type_func=0`; a later validation set `alpha_lidar=1.0` for the committed profile.

Why it changed:

- The previous optimizer did not tune the gamma/delta wheel covariance path because `wheel_type_func=1` bypassed those parameters.
- In adaptive wheel mode, the filter computes wheel covariance from yaw-rate disagreement: `gamma_vx * |wz_wheel - wz_imu| + delta_vx` and `gamma_omegaz * |wz_wheel - wz_imu| + delta_omegaz`.
- This matches the robot physics better than fixed covariance: wheel odometry can help on straight segments but should lose authority during turns.
- The bag tests still showed that even adaptive wheel fusion can hurt global consistency, while the LiDAR-only filter provided the best cross-bag mean score.

Experiment results:

- Best adaptive-wheel candidate on bags 4, 5, and 6 used `enableWheel=true`, `wheel_type_func=0`, `lidarG=0.5`, `alpha_lidar=0.998`, `wheelOffset=0.05`, `gamma_vx=3.0`, `gamma_omegaz=80.0`, `delta_vx=0.05`, and `delta_omegaz=2.0`.
- That adaptive-wheel candidate reached mean XY RMSE `0.2034 m` on bags 4, 5, and 6, with FAST-LIO2 at `0.1743 m`; it beat FAST-LIO2 on bag6 but failed bag3 validation with `0.7590 m`.
- The best LiDAR-only candidate across bags 3, 4, 5, and 6 used `enableWheel=false`, `enableImu=false`, `lidarG=0.5`, and `alpha_lidar=0.998`.
- That LiDAR-only candidate reached mean XY RMSE `0.1999 m` across all four bags, while FAST-LIO2 measured `0.2268 m` on the same path-only metric.
- The LiDAR-only candidate beat FAST-LIO2 on two of the four bags and gave the best overall aggregate score among the tested configurations.

What to watch next:

- Test the LiDAR-only profile first on the robot to confirm the wheel correction was the source of the weird path behavior.
- If LiDAR degrades in feature-poor robot runs, re-enable wheel with the adaptive-wheel candidate above and record whether turns or feature-poor straight segments cause the failure.
- Record meaningful IMU angular velocity and covariance; adaptive wheel mode can use IMU yaw rate as a wheel reliability signal even when IMU orientation correction stays disabled.

## 2026-05-12

### Filter output source gating investigation

Files changed:

- `include/adaptive_odom_filter/ekf_adaptive_tools.h`
- `src/ekf_adaptive_tools.cpp`
- `src/EKFAdaptiveFilter.cpp`
- `CHANGELOG.md`

What changed:

- The filter output cache now honors the existing `filterFreq` parameter.
- With the current `filterFreq: "l"` default, the node caches and publishes only LiDAR-corrected output states instead of interleaving IMU, wheel, and LiDAR correction states.
- A pose-row Kalman gain suppression hypothesis was tested and removed after a clean replay showed the blink came from topic contamination instead.

Why it changed:

- Fresh bag playback first appeared to show `/filter_odom` jumping by more than `3 m` in adjacent published messages.
- A clean replay to `/filter_odom_live` showed no adjacent jumps above `0.5 m`, proving the first recording mixed the old bagged `/filter_odom` with the live filter output.
- The original bag contains 1,416 old `/filter_odom` messages; replaying it while publishing the live filter on `/filter_odom` makes RViz alternate between two filter sources.
- The package already had `filterFreq: "l"` in the parameters, but the ROS 2 adaptive node did not use it when selecting the output state.

Validation and test context:

- `bag3_odom_ros2dev_010526` was replayed with the live filter remapped to `/filter_odom_live`.
- The clean live filter path had 1,425 messages, no adjacent XY step above `0.5 m`, and a path length of `70.24 m` against the `/scout_mini/Odom` path length of `70.73 m`.
- A no-IMU diagnostic ran because the bag IMU reports zero orientation and angular-velocity covariances.
- The no-IMU run improved path-only RMSE against `/scout_mini/Odom` from `0.485 m` to `0.239 m`, while FAST-LIO2 alone measured `0.358 m` on the same path-only metric.
- A balanced wheel/LiDAR covariance candidate (`lidarG=5.0`, `wheelGVx=0.02`, `wheelGVy=0.05`, `wheelGWz=0.05`) worsened path-only RMSE to `0.767 m`; the current wheel/LiDAR gains performed better for this bag.

### ROS 2 adaptive EKF correction and bag-test tuning

Files changed:

- `include/adaptive_odom_filter/ekf_adaptive_tools.h`
- `src/ekf_adaptive_tools.cpp`
- `src/EKFAdaptiveFilter.cpp`
- `config/adaptive_filter_parameters.yaml`
- `launch/adaptive_filter_fastlio_wheelodom.launch.py`
- `CHANGELOG.md`

What changed:

- The indirect odometry measurement model is now pure, so the numerical Jacobian code no longer mutates the alpha-beta filter state while it probes the model.
- Alpha-beta smoothing remains in the LiDAR and visual correction path, but the correction applies it exactly once for each real correction.
- The LiDAR correction now distinguishes pose covariance from twist covariance. When Fast-LIO2 publishes pose covariance, the filter propagates it through the pose-delta-to-body-velocity Jacobians; when the node falls back to twist covariance, the filter uses it directly.
- Covariance-mode bookkeeping for the current and previous LiDAR samples keeps a consistent interpretation for both sides of the finite-difference velocity estimate.
- The EKF worker thread now tracks a corrected-state sequence number and last-update source.
- Filtered odometry publication moved out of the LiDAR callback and into a timer, so the ROS 2 node publishes only states that the filter thread has already predicted and corrected.
- A mutex now protects output headers, which keeps the timer publisher and sensor callbacks from racing on the selected header.
- `get_state()` now returns the cached output state instead of reading the mutable internal EKF state directly.
- The hard-coded wheel velocity conversion `-0.705 * /odom.twist.twist.linear.x` was replaced with the `wheelVxScale` parameter.
- `wheelVxScale: 1.0` keeps the default wheel `vx` path from flipping or shrinking `/odom.twist.twist.linear.x`.
- `wheelGVx` changed from `0.001` to `0.1`, so wheel longitudinal velocity no longer dominates the LiDAR-derived velocity as aggressively after the LiDAR covariance fix.
- `adaptive_filter_fastlio_wheelodom.launch.py` sets `use_sim_time` to `true` for bag playback tests.

Why it changed:

- The previous Jacobian path updated the alpha-beta filter during finite-difference probes, which could corrupt the correction input before the EKF used it.
- The previous LiDAR covariance path treated pose covariance too much like velocity covariance, which made LiDAR confidence inconsistent with the measurement model.
- The previous callback publication path could publish a state before the worker thread finished the correction cycle.
- The previous wheel `vx` sign and scale made the filtered position look mirrored while orientation still looked correct when `/odom` already used the expected forward convention.
- The previous `wheelGVx` value trusted wheel forward velocity strongly enough to shrink the filtered lemniscate when wheel scale/sign disagreed with Fast-LIO2 or ground truth.

Validation and test context:

- Static source review and `git diff --check` ran.
- The implementation phase did not build or launch the package locally because the test plan requested code-only changes.
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
