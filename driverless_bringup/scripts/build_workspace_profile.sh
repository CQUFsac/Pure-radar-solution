#!/usr/bin/env bash
set -euo pipefail

if [[ $# -ne 1 || ("$1" != "scout" && "$1" != "fssim") ]]; then
  echo "Usage: $0 scout|fssim"
  exit 2
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SRC_DIR="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
WS_DIR="$(cd "${SRC_DIR}/.." && pwd)"
PROFILE="$1"

enable_package()
{
  local relative_path="$1"
  if [[ -f "${SRC_DIR}/${relative_path}/CATKIN_IGNORE" ]]; then
    rm -f "${SRC_DIR}/${relative_path}/CATKIN_IGNORE"
  fi
}

# The teammate localization tree contains an older package with the same ROS
# package name. Keep it disabled; the maintained detector is in the perception
# repository.
touch "${SRC_DIR}/estimation/lidar_cone_detector/CATKIN_IGNORE"

if [[ "${PROFILE}" == "scout" ]]; then
  enable_package "lidar_cone_detector/lidar_cone_detector"
  enable_package "lidar_cone_detector/roi_filter"
  enable_package "lidar_cone_detector/rslidar_sdk"
  enable_package "estimation/robot_localization"

  RSLIDAR_DIR="${SRC_DIR}/lidar_cone_detector/rslidar_sdk"
  if [[ ! -e "${RSLIDAR_DIR}/package.xml" &&
        -f "${RSLIDAR_DIR}/package_ros1.xml" ]]; then
    ln -s package_ros1.xml "${RSLIDAR_DIR}/package.xml"
  fi

  PACKAGES="driverless_msgs;lidar_cone_detector;roi_filter;rslidar_sdk;local_path_planner;scout_path_controller;driverless_bringup;robot_localization;scout_base;scout_bringup;scout_description;scout_msgs;ugv_sdk;wrp_io"
  echo "Building SCOUT MINI profile"
else
  PACKAGES="cone_car_core;driverless_msgs;local_path_planner;fssim_path_controller;driverless_bringup;fssim_common;fssim_description;fssim_gazebo;fssim_gazebo_plugins;fssim"
  echo "Building FSSIM profile"
fi

cd "${WS_DIR}"
catkin_make -DCATKIN_WHITELIST_PACKAGES="${PACKAGES}"

echo
echo "Build complete. Run:"
echo "  source ${WS_DIR}/devel/setup.bash"
if [[ "${PROFILE}" == "scout" ]]; then
  echo "  roslaunch driverless_bringup scout_mini_algorithm.launch"
else
  echo "  roslaunch driverless_bringup fssim_algorithm.launch"
fi
