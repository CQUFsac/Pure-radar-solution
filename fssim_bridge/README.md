# fssim_bridge

`fssim_bridge` is the only active FSSIM integration package in this workspace.
The old `fssim_test` package is kept only as a backup and is disabled with
`CATKIN_IGNORE`.

The supported simulator is:

- https://github.com/Huguet57/fssim-2021
- Ubuntu 20.04
- ROS Noetic
- Gazebo 11

## Data flow

```text
/lidar/cones (PointCloud2, fssim/vehicle/base_link)
  -> fssim_bridge_node
  -> /perception/lidar/cones_raw (ConeObservationArray, fssim/vehicle/base_link)
  -> local_path_planner
  -> /planning/local_path (nav_msgs/Path, base_link)
  -> Pure Pursuit + speed controller
  -> /fssim/cmd (fssim_common/Cmd)

/fssim/base_pose_ground_truth (fssim_common/State)
  -> fssim_odom_bridge
  -> /odometry/filtered + TF(fssim_map -> base_link)
```

The planner uses the native TF
`fssim/vehicle/base_link -> fssim_map -> base_link` to transform delayed cone
observations into the stack's `base_link` frame. Do not publish a second TF with
the same child frame.

FSSIM does not provide raw LiDAR scans. `/lidar/cones` is a simulated cone
observation model, so ROI filtering, ground segmentation and clustering are not
tested by this simulation. Its custom PointCloud2 also contains simulated
blue/yellow/orange probabilities. The FSSIM launch enables those labels only
for simulator path-planning tests; the real pure-LiDAR profile continues to
publish `UNKNOWN`.

## Build and run

```bash
cd ~/fase_ws_sim
catkin build driverless_msgs local_path_planner fssim_bridge
source devel/setup.bash
roslaunch fssim_bridge fssim_integration.launch
```

Wait until the terminal prints `Sending RES GO`. FSSIM ignores drive commands
before RES GO.

If FSSIM is already running:

```bash
roslaunch fssim_bridge fssim_integration.launch start_fssim:=false
```

If steering is reversed:

```bash
roslaunch fssim_bridge fssim_integration.launch steering_sign:=-1.0
```

## First checks when the path is empty

```bash
rostopic hz /lidar/cones
rostopic echo -n 1 /lidar/cones/header
rostopic hz /fssim/base_pose_ground_truth
rostopic hz /odometry/filtered
rostopic echo /planning/path_status
rostopic hz /planning/local_path
rostopic echo /fssim_bridge/status
rostopic echo /fssim/cmd
```

Expected rates:

- `/lidar/cones`: about 10 Hz
- `/odometry/filtered`: faster than 10 Hz
- `/planning/local_path`: about 10 Hz while enough cones are visible
- `/fssim/cmd`: 20 Hz

Bridge status:

- `RUNNING`: a fresh valid path is being tracked
- `RUNNING_ON_LAST_PATH`: the planner reports degraded history or the latest
  planning frame is invalid; the planner motion-compensates the previous path
  and the vehicle follows it at 30% target speed for at most about 1.0 s
- `WAITING_FOR_PATH`: no valid path has been available for 0.6 s, so the bridge
  commands a stop
- `CREEPING_FOR_CONES`: cone messages are alive but a short gap cannot produce
  a new path. The controller follows the odometry-compensated last path for no
  more than 3.5 m or 5.0 s, then stops.

The simulator planner triangulates all cones. A blue-yellow edge creates the
normal cross-track midpoint. A short same-colour boundary edge can also create
a virtual centre point by shifting half the expected track width inward. This
keeps the path alive when the opposite cone is missing.

Same-colour recovery is not raw nearest-neighbour chaining: link length,
heading agreement at both endpoints, boundary side, sampled interior offsets,
path-progress difference and distance to the odometry-compensated previous path
are gated. Each cone may have at most two same-colour boundary links. These
checks prevent the nearest cone on the return branch of a hairpin from being
selected merely because its Euclidean distance is small.

In the planner debug markers, normal cross-track candidate edges are blue and
same-colour boundary-recovery edges are magenta.

When both boundary recovery and ordinary pairing leave two disconnected
centre-line sections, the FSSIM profile may bridge one gap up to 8.5 m. This
long bridge is enabled only when an odometry-compensated previous path exists.
Candidate progress must move forward along that reference path and may not
backtrack into a nearby hairpin branch.

## Initial parameters

All values below are initial simulation values and must be tuned:

- `wheelbase`: 1.53 m, from the Gotthard FSSIM model
- `lookahead_distance`: 2.0 m
- `cruise_speed`: 3.0 m/s
- `minimum_corner_speed`: 1.0 m/s
- `max_steer`: 0.40 rad
- `max_drive_command`: 0.20
- `max_brake_command`: -0.20

`dc` is a drivetrain command, not vehicle speed in m/s. The controller therefore
uses odometry feedback to convert a target speed into `dc`.
