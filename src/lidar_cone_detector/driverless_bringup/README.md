# Driverless Bringup Profiles

All team modules may remain under the same `fase_ws_sim/src` directory. Build
only one platform profile at a time:

```bash
cd ~/fase_ws_sim
bash src/lidar_cone_detector/driverless_bringup/scripts/build_workspace_profile.sh scout
# or
bash src/lidar_cone_detector/driverless_bringup/scripts/build_workspace_profile.sh fssim
```

Two algorithm-only launch profiles are provided:

```bash
roslaunch driverless_bringup scout_mini_algorithm.launch
roslaunch driverless_bringup fssim_algorithm.launch
```

SCOUT fixed-event profiles:

```bash
roslaunch driverless_bringup scout_acceleration.launch
roslaunch driverless_bringup scout_skidpad.launch
```

Both fixed-event profiles enable the mission direction/speed/stop interface.
See `docs/SCOUT_FIXED_EVENTS.md` before a real-vehicle test.

Teammate IMU and cone-map integration is documented in
`docs/TEAM_CODE_INTEGRATION.md`. Mapping is optional and does not block the
pure-local-LiDAR driving path.

They intentionally do not start hardware drivers, FSSIM itself, TF bridges, or
localization packages. Those components are platform-specific and must satisfy
the input contracts documented in `docs/TWO_PLATFORM_ARCHITECTURE.md`.
