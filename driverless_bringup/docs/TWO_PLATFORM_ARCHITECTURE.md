# SCOUT MINI and Formula Student platform split

The perception and planning layers are shared. Vehicle-state configuration and
the control adapter are platform-specific.

## Shared pipeline

```text
non-ground cloud
-> lidar_cone_detector
-> /perception/lidar/cones_raw
-> cone tracker (when available)
-> /perception/cones
-> local_path_planner
-> /planning/local_path
```

Until the tracker is available, the planner may subscribe directly to
`/perception/lidar/cones_raw`.

## Profile A: SCOUT MINI

```text
SCOUT wheel odometry + CH110 IMU
-> robot_localization (two_d_mode)
-> /odometry/filtered

/planning/local_path + /odometry/filtered
-> scout_path_controller
-> geometry_msgs/Twist
-> /cmd_vel
```

The controller uses `angular.z = linear.x * curvature`. It does not expose a
front-wheel steering angle.

Only one node may publish `odom -> base_link`. Cone scan matching or cone SLAM
may later publish `map -> odom`, but never simultaneously.

## Profile B: FSSIM and future Formula Student car

```text
/planning/local_path + /odometry/filtered
-> fssim_path_controller
-> fssim_common/Cmd(dc, delta)
-> /fssim/cmd
```

`dc` is a simulator-normalized drive/brake command. It must not be sent
directly to a real vehicle.

For a real Formula Student car, keep perception, mapping and planning, then
replace `fssim_path_controller` with a CAN control adapter that converts target
speed and curvature into:

- front-wheel steering request;
- drive torque request;
- brake pressure request.

The teammate's bicycle-model EKF belongs to this profile. The SCOUT profile
must use a differential-drive/planar EKF configuration instead.

## Required interface unification

The perception repository currently uses:

```text
driverless_msgs/ConeObservationArray
/perception/lidar/cones_raw
```

Some localization packages use:

```text
cone_car_core/ConeArray
/perception/cones
```

Use a temporary adapter if required, then migrate localization to
`driverless_msgs/ConeObservationArray`. Do not keep two permanent cone message
definitions.

The preprocessor must provide:

```text
/lidar/non_ground_points
/lidar/ground_points
/lidar/preprocessing_status
```

The existing ROI-only node publishes different topic names and uses a simple
height threshold. It is not the final ground segmenter for outdoor SCOUT tests.

## One workspace, two build profiles

FSSIM packages depend on `fssim_common`; SCOUT hardware workspaces normally do
not. The repository therefore uses a catkin package whitelist inside one
workspace:

```text
fase_ws_sim/src
├── lidar_cone_detector/       shared perception, planning and controllers
├── estimation/               teammate localization and mapping
├── cone_car_core/             teammate common messages
├── scout_base/                SCOUT ROS driver and messages
├── ugv_sdk/                   SCOUT CAN/serial SDK
├── fssim*/                    simulator packages
└── ...
```

Build commands:

```bash
bash src/lidar_cone_detector/driverless_bringup/scripts/build_workspace_profile.sh scout
bash src/lidar_cone_detector/driverless_bringup/scripts/build_workspace_profile.sh fssim
```

The script keeps `estimation/lidar_cone_detector` disabled because it has the
same ROS package name as the maintained detector. It does not launch both
vehicle controllers or both TF publishers.

The existing generic `serial_imu` package is not assumed to be a CH110 driver.
Confirm the CH110 protocol, baud rate, message format and timestamp behavior
before adding it to the SCOUT profile.
