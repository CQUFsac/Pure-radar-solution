# FSSIM Path Controller

This package contains only local-path tracking and FSSIM vehicle command output.
It does not start FSSIM, convert cone messages, publish odometry TF, or configure
RViz.

Inputs:

- `/planning/local_path` (`nav_msgs/Path`)
- `/planning/path_status` (`std_msgs/String`)
- `/odometry/filtered` (`nav_msgs/Odometry`)

Outputs:

- `/fssim/cmd` (`fssim_common/Cmd`)
- `/fssim_path_controller/status` (`std_msgs/String`)

The controller uses Pure Pursuit, curvature-based speed reduction, dynamic
lookahead, steering rate limiting, and a distance-limited recovery for short
cone-pair gaps.

Initial parameters are stored in `config/controller.yaml` and must be confirmed
with the selected FSSIM vehicle model.
