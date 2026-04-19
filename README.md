# adaptive_odom_filter

ROS 2 package for odometry fusion using wheel odometry, IMU, LiDAR odometry, and optional visual odometry.

The package provides two filter pipelines:

- `adaptive_odom_filter_node` / `EKFAdaptiveFilter`: adaptive EKF for wheel + IMU + LiDAR, with optional visual odometry
- `EKFRobustAdaptiveFilter`: robust EKF path with extended state and sensor extrinsic outputs

The ROS 2 branch keeps the package structure, launch flow, and runtime integration in ROS 2, while porting the latest filter logic that was added in `master`.

## Package layout

- `src/EKFAdaptiveFilter.cpp`: ROS 2 adaptive EKF node
- `src/EKFRobustAdaptiveFilter.cpp`: ROS 2 robust EKF node
- `src/ekf_adaptive_tools.cpp`: adaptive EKF core
- `src/adaptive_robust_ekf.cpp`: robust EKF core
- `config/adaptive_filter_parameters.yaml`: adaptive filter parameters
- `config/adaptive_robust_filter_parameters.yaml`: robust filter parameters
- `launch/*.launch.py`: ROS 2 launch entry points

## Build

Place the package inside a ROS 2 workspace `src/` folder and build with `colcon`:

```bash
colcon build --symlink-install
```

Source the workspace after a successful build:

```bash
source install/setup.bash
```

## Main dependencies

- ROS 2 Jazzy or compatible ament workspace
- `rclcpp`
- `nav_msgs`
- `sensor_msgs`
- `geometry_msgs`
- `tf2`, `tf2_ros`, `tf2_geometry_msgs`
- `rtabmap_msgs`
- `Eigen3`

Optional:

- `cloud_msgs`
  The robust node can subscribe to LiDAR feature counts when the ROS 2 `cloud_msgs` package is available.

## Running the package

Adaptive EKF:

```bash
ros2 launch adaptive_odom_filter adaptive_filter_odom.launch.py
```

Robust EKF:

```bash
ros2 launch adaptive_odom_filter adaptive_robust_filter_odom.launch.py
```

Fast test path with fake publishers:

```bash
ros2 launch adaptive_odom_filter adaptive_filter_odom.launch.py test:=true
```

## Inputs and outputs

### Adaptive EKF inputs

Default topics are configured in `config/adaptive_filter_parameters.yaml`.

- IMU: `/imu/data`
- wheel odometry: `/odom_with_cov`
- LiDAR odometry: `/ekf_loam/laser_odom_with_cov`
- tracking visual odometry: `/camera/odom`
- depth visual odometry: `/camera/odom`
- stereo / RGB image topics for visual confidence estimation

### Adaptive EKF output

- fused odometry: `/ekf_loam/filter_odom_to_init`

### Robust EKF inputs

Default topics are configured in `config/adaptive_robust_filter_parameters.yaml`.

- IMU: `/imu/data`
- wheel odometry: `/odom_with_cov`
- LiDAR odometry: `/ekf_loam/laser_odom_with_cov`
- LiDAR features: `/ekf_loam/features_cloud_info`

### Robust EKF outputs

- fused odometry: `/ekf_loam/filter_odom_to_init`
- LiDAR extrinsic pose: `/lidar_extrinsic_pose`
- IMU extrinsic pose: `/imu_extrinsic_pose`

## Key parameters to tune

### Common enable flags

- `enableFilter`
- `enableImu`
- `enableWheel`
- `enableLidar`
- `enableVisual`

Use these first to isolate sensors during bring-up and debugging.

### Timing and output behavior

- `freq`
  Filter update frequency.
- `filterFreq`
  Legacy output selection flag kept for compatibility.

If the filter becomes noisy or unstable under load, reduce `freq` first and validate sensor timestamps.

### Adaptive EKF sensor gains

- `lidarG`
  Global LiDAR covariance scaling.
- `imuG`
  IMU orientation covariance scaling.
- `visualG`
  Visual odometry covariance scaling.
- `wheelGVx`
- `wheelGVy`
- `wheelGWz`
- `wheelOffset`

The split wheel gains are important in the ROS 2 branch because wheel velocity and yaw-rate are handled independently.

### Wheel adaptive covariance parameters

- `gamma_vx`
- `gamma_omegaz`
- `delta_vx`
- `delta_omegaz`
- `wheel_type_func`

These parameters affect how strongly wheel covariance reacts to disagreement between wheel odometry and IMU yaw rate.

### LiDAR / visual covariance modes

- `lidar_type_func`
- `visual_type_func`
- `alpha_lidar`
- `alpha_visual`

These control indirect odometry smoothing and covariance selection.

### Adaptive EKF experiment mode

- `experiment`

This parameter is used to switch covariance experiments in the adaptive path. Keep it at `0` unless you are intentionally testing covariance weighting behavior.

### Robust EKF parameters

Important robust-path parameters are defined in `config/adaptive_robust_filter_parameters.yaml`:

- `lidarG`
- `wheelG`
- `imuG`
- `gamma_vx`
- `gamma_omegaz`
- `delta_vx`
- `delta_omegaz`
- `lidar_type_func`
- `wheel_type_func`

The robust path uses a different state model and fixed sensor covariance handling in key callbacks, so tune it independently from the adaptive path.

## Tuning guide

Recommended order:

1. Validate timestamps and frames first.
2. Run with `enableVisual:=false` unless visual odometry is already known to be stable.
3. Tune `lidarG` and `imuG` before touching wheel adaptive terms.
4. Tune `wheelGVx`, `wheelGVy`, and `wheelGWz` separately.
5. Only then adjust `gamma_*` and `delta_*`.

Symptoms and likely causes:

- fused odometry follows wheel motion too aggressively
  Lower wheel gains or increase wheel covariance terms.
- fused odometry ignores wheel constraints
  Increase wheel gains or reduce LiDAR dominance.
- yaw oscillation or fast heading jumps
  Recheck wheel sign convention, IMU orientation covariance, and `imuG`.
- filter stalls waiting for data
  Check sensor timestamps and whether the enabled inputs are actually publishing.

## Common build and runtime issues

### CMake cache path mismatch

Error example:

```text
CMake Error: The current CMakeCache.txt directory ... is different than the directory ... where CMakeCache.txt was created
```

Cause:

- stale generated `build/`, `install/`, or `log/` content from another machine or workspace path

Fix:

```bash
rm -rf build install log
colcon build --symlink-install
```

This branch now keeps generated artifacts out of version control, so a fresh clone should not hit this by default.

### Missing `cloud_msgs`

If the workspace does not provide ROS 2 `cloud_msgs`, the package still builds, but the robust node will not receive LiDAR feature count messages from that package.

### Topic mismatch

If the node starts but the filter does not move, check:

- topic names in the YAML files
- topic names in the launch file arguments
- actual published topic names in the robot workspace

### Frame mismatch

If the fused odometry is valid numerically but wrong in downstream consumers, check:

- `odom_frame_id`
- `base_frame_id`
- the frames used by your wheel, IMU, and LiDAR pipelines

## Notes

- The robust path publishes estimated sensor extrinsics from the filter state.
- The adaptive path supports optional visual odometry, but visual inputs should only be enabled after topic, covariance, and timing validation.
- The package includes a fake publisher script for fast integration testing, not for performance benchmarking.

## Change tracking

Branch-level changes are recorded in [CHANGELOG.md](./CHANGELOG.md).
