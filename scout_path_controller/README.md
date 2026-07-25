# SCOUT MINI Path Controller

This package is the differential-drive control adapter for the current
SCOUT MINI validation platform.

Inputs:

- `/planning/local_path` (`nav_msgs/Path`, normally in `base_link`)
- `/planning/path_status` (`std_msgs/String`)
- `/odometry/filtered` (`nav_msgs/Odometry`)

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
