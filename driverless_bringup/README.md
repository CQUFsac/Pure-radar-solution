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

They intentionally do not start hardware drivers, FSSIM itself, TF bridges, or
localization packages. Those components are platform-specific and must satisfy
the input contracts documented in `docs/TWO_PLATFORM_ARCHITECTURE.md`.
