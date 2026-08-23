cmake_minimum_required(VERSION 3.0.2)
project(fssim_bridge)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  driverless_msgs
  fssim_common
  geometry_msgs
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
)

catkin_package(
  CATKIN_DEPENDS
    driverless_msgs
    fssim_common
    geometry_msgs
    nav_msgs
    roscpp
    sensor_msgs
    std_msgs
    tf2
    tf2_ros
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(fssim_bridge_node src/fssim_bridge_node.cpp)
target_link_libraries(fssim_bridge_node ${catkin_LIBRARIES})
add_dependencies(fssim_bridge_node ${catkin_EXPORTED_TARGETS})

add_executable(fssim_odom_bridge src/fssim_odom_bridge.cpp)
target_link_libraries(fssim_odom_bridge ${catkin_LIBRARIES})
add_dependencies(fssim_odom_bridge ${catkin_EXPORTED_TARGETS})

# Simulator-only skidpad reference generator.  It is deliberately kept in
# fssim_bridge so the real-vehicle Delaunay planner remains map independent.
add_executable(fssim_skidpad_path_node src/fssim_skidpad_path_node.cpp)
target_link_libraries(fssim_skidpad_path_node ${catkin_LIBRARIES})
add_dependencies(fssim_skidpad_path_node ${catkin_EXPORTED_TARGETS})

install(TARGETS
  fssim_bridge_node
  fssim_odom_bridge
  fssim_skidpad_path_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
