cmake_minimum_required(VERSION 3.0.2)
project(fssim_test)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2_ros
  fssim_common
  driverless_msgs
  cone_car_core
  local_path_planner
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs nav_msgs sensor_msgs
                 tf2_ros fssim_common driverless_msgs cone_car_core
                 local_path_planner
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Odometry bridge: fssim/state → /odometry/filtered
add_executable(fssim_odom_bridge src/fssim_odom_bridge.cpp)
target_link_libraries(fssim_odom_bridge ${catkin_LIBRARIES})
add_dependencies(fssim_odom_bridge ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(cone_topic_converter src/cone_topic_converter.cpp)
target_link_libraries(cone_topic_converter ${catkin_LIBRARIES})
add_dependencies(cone_topic_converter ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

install(TARGETS fssim_odom_bridge cone_topic_converter
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
