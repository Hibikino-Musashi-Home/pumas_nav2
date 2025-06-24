cmake_minimum_required(VERSION 3.5)
project(tmc_manipulation_types_bridge)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tmc_manipulation_types REQUIRED)
find_package(tmc_planning_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

add_library(manipulation_msg_convertor SHARED src/manipulation_msg_convertor.cpp)
target_include_directories(manipulation_msg_convertor PUBLIC include ${EIGEN3_INCLUDE_DIRS})
ament_target_dependencies(manipulation_msg_convertor nav_msgs rclcpp tf2_eigen tmc_manipulation_types tmc_planning_msgs visualization_msgs)

install(TARGETS manipulation_msg_convertor
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(manipulation_msg_convertor_test test/manipulation_msg_convertor-test.cpp)
  target_link_libraries(manipulation_msg_convertor_test manipulation_msg_convertor)
endif()

ament_export_include_directories(include)
ament_export_libraries(manipulation_msg_convertor)
ament_export_dependencies(nav_msgs rclcpp tf2_eigen tmc_manipulation_types tmc_planning_msgs visualization_msgs)

ament_package()
