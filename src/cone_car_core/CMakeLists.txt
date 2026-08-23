cmake_minimum_required(VERSION 3.0.2)
project(cone_car_core)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosbag
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf
  visualization_msgs
  message_generation
)

# ============================================================
# Custom Messages
# ============================================================
add_message_files(
  FILES
  Cone.msg
  ConeArray.msg
  VehicleState.msg
  PathPoint.msg
  PlanningPath.msg
  ModeStatus.msg
  ControlCommand.msg
  ChassisStatus.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

# ============================================================
# catkin_package
# ============================================================
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    roscpp
    rosbag
    std_msgs
    geometry_msgs
    sensor_msgs
    nav_msgs
    tf
    visualization_msgs
    message_runtime
)

# ============================================================
# Include directories
# ============================================================
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# ============================================================
# Perception: fake_cone_publisher
# ============================================================
add_executable(fake_cone_publisher src/perception/fake_cone_publisher.cpp)
target_link_libraries(fake_cone_publisher ${catkin_LIBRARIES})
add_dependencies(fake_cone_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# ============================================================
# Planning: path_generator_node
# ============================================================
add_executable(path_generator_node src/planning/path_generator_node.cpp)
target_link_libraries(path_generator_node ${catkin_LIBRARIES})
add_dependencies(path_generator_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# ============================================================
# Mode: mode_manager_node
# ============================================================
add_executable(mode_manager_node src/mode/mode_manager_node.cpp)
target_link_libraries(mode_manager_node ${catkin_LIBRARIES})
add_dependencies(mode_manager_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# ============================================================
# Control: pure_pursuit_controller_node
# ============================================================
add_executable(pure_pursuit_controller_node src/control/pure_pursuit_controller_node.cpp)
target_link_libraries(pure_pursuit_controller_node ${catkin_LIBRARIES})
add_dependencies(pure_pursuit_controller_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# ============================================================
# Vehicle: fake_vehicle_node
# ============================================================
add_executable(fake_vehicle_node src/vehicle/fake_vehicle_node.cpp)
target_link_libraries(fake_vehicle_node ${catkin_LIBRARIES})
add_dependencies(fake_vehicle_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# ============================================================
# Logger: data_logger_node
# ============================================================
add_executable(data_logger_node src/logger/data_logger_node.cpp)
target_link_libraries(data_logger_node ${catkin_LIBRARIES})
add_dependencies(data_logger_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# ============================================================
# Visualization: rviz_visualizer_node
# ============================================================
add_executable(rviz_visualizer_node src/visualization/rviz_visualizer_node.cpp)
target_link_libraries(rviz_visualizer_node ${catkin_LIBRARIES})
add_dependencies(rviz_visualizer_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

