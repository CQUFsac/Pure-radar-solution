cmake_minimum_required(VERSION 3.0.2)
project(fssim_path_controller)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  fssim_common
  nav_msgs
  roscpp
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    fssim_common
    nav_msgs
    roscpp
    std_msgs
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(
  fssim_path_controller_node
  src/fssim_path_controller_node.cpp
)
target_link_libraries(
  fssim_path_controller_node
  ${catkin_LIBRARIES}
)
add_dependencies(
  fssim_path_controller_node
  ${catkin_EXPORTED_TARGETS}
)

install(
  TARGETS fssim_path_controller_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(
  DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
