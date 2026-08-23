cmake_minimum_required(VERSION 3.0.2)
project(serial_imu LANGUAGES C CXX)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  serial
)

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs serial
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(
  serial_imu_node
  src/serial_imu.cpp
  src/imu_data_decode.c
  src/packet.c
)
target_link_libraries(serial_imu_node ${catkin_LIBRARIES})
add_dependencies(serial_imu_node ${catkin_EXPORTED_TARGETS})

install(
  TARGETS serial_imu_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(
  DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
