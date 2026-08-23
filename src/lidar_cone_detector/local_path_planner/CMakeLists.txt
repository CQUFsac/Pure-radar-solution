cmake_minimum_required(VERSION 3.0.2)
project(local_path_planner)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  driverless_msgs
  geometry_msgs
  nav_msgs
  roscpp
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}_lib
  CATKIN_DEPENDS
    driverless_msgs
    geometry_msgs
    nav_msgs
    roscpp
    std_msgs
    tf2
    tf2_geometry_msgs
    tf2_ros
    visualization_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_lib
  src/candidate_edge_generator.cpp
  src/path_searcher.cpp
  src/path_smoother.cpp
  src/path_validator.cpp
  src/single_side_recovery.cpp
)
target_link_libraries(${PROJECT_NAME}_lib ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}_lib ${catkin_EXPORTED_TARGETS})

add_executable(${PROJECT_NAME}_node src/local_path_planner_node.cpp)
target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}_lib
  ${catkin_LIBRARIES}
)
add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_local_path_planner test/test_local_path_planner.cpp)
  if(TARGET test_local_path_planner)
    target_link_libraries(test_local_path_planner ${PROJECT_NAME}_lib ${catkin_LIBRARIES})
  endif()
endif()

install(TARGETS ${PROJECT_NAME}_lib ${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
