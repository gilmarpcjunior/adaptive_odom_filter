# Changelog

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
