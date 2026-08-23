cmake_minimum_required(VERSION 3.0.2)
project(scout_mission_manager)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  driverless_msgs
  geometry_msgs
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
  std_srvs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

catkin_package(
  CATKIN_DEPENDS
    driverless_msgs
    geometry_msgs
    nav_msgs
    roscpp
    sensor_msgs
    std_msgs
    std_srvs
    tf2
    tf2_geometry_msgs
    tf2_ros
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(
  scout_mission_manager_node
  src/scout_mission_manager_node.cpp
)
target_link_libraries(
  scout_mission_manager_node
  ${catkin_LIBRARIES}
)
add_dependencies(
  scout_mission_manager_node
  ${catkin_EXPORTED_TARGETS}
)

install(
  TARGETS scout_mission_manager_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(
  DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
