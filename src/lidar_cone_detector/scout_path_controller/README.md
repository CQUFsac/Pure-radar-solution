# SCOUT MINI Path Controller

This package is the differential-drive control adapter for the current
SCOUT MINI validation platform.

Inputs:

- `/planning/local_path` (`nav_msgs/Path`, normally in `base_link`)
- `/planning/path_status` (`std_msgs/String`)
- `/odom` or `/odometry/filtered` (`nav_msgs/Odometry`, optional)
- `/mission/speed_limit` (`std_msgs/Float32`, optional)
- `/mission/stop` (`std_msgs/Bool`, optional)

Outputs:

- `/cmd_vel` (`geometry_msgs/Twist`)
- `/scout_path_controller/status` (`std_msgs/String`)

The controller computes path curvature with Pure Pursuit and sends:

```text
linear.x  = v
angular.z = v * curvature
```

It does not calculate or output a front-wheel steering angle. All parameters in
`config/controller.yaml` are initial suggestions and require low-speed vehicle
tests.

The current profile defaults to `use_odometry: false`, so the controller can
run from the current local path without either odometry topic. `/odom`,
`/odometry/filtered`, path motion compensation and measured-distance recovery
remain available for later use by setting `use_odometry: true`.

The mission inputs are disabled by default. The acceleration and skidpad
bringup profiles enable them. When enabled, missing mission messages or
`/mission/stop=true` force a zero `Twist`; the speed limit only reduces the
controller target and never raises the configured maximum speed.
