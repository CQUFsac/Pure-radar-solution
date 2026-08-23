cmake_minimum_required(VERSION 3.0.2)
project(cone_map_manager)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  driverless_msgs
  geometry_msgs
  roscpp
  std_msgs
  std_srvs
  tf2
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
)

catkin_package(
  CATKIN_DEPENDS
    driverless_msgs
    geometry_msgs
    roscpp
    std_msgs
    std_srvs
    tf2
    tf2_geometry_msgs
    tf2_ros
    visualization_msgs
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(
  cone_map_manager_node
  src/cone_map_manager_node.cpp
)
target_link_libraries(
  cone_map_manager_node
  ${catkin_LIBRARIES}
)
add_dependencies(
  cone_map_manager_node
  ${catkin_EXPORTED_TARGETS}
)

install(
  TARGETS cone_map_manager_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(
  DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
