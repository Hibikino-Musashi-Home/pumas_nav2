cmake_minimum_required(VERSION 3.5)
project(tmc_safe_pose_changer)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(angles REQUIRED)
find_package(control_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tmc_manipulation_util REQUIRED)
find_package(tmc_planning_msgs REQUIRED)
find_package(tmc_utils REQUIRED)
find_package(trajectory_msgs REQUIRED)


add_library(${PROJECT_NAME} SHARED src/safe_pose_changer.cpp)
ament_target_dependencies(${PROJECT_NAME}
  angles
  control_msgs
  geometry_msgs
  rclcpp
  rclcpp_action
  sensor_msgs
  tmc_manipulation_util
  tmc_planning_msgs
  tmc_utils
  trajectory_msgs
)

add_executable(safe_pose_changer src/safe_pose_changer_node.cpp)
target_link_libraries(safe_pose_changer ${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME} safe_pose_changer
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(safe_pose_changer_test test/safe_pose_changer-test.cpp)
  target_link_libraries(safe_pose_changer_test ${PROJECT_NAME})
endif()

ament_package()
