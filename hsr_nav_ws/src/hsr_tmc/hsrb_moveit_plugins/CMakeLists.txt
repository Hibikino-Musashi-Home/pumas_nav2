cmake_minimum_required(VERSION 3.10.2)
project(hsrb_moveit_plugins)

find_package(ament_cmake REQUIRED)
find_package(hsrb_analytic_ik REQUIRED)
find_package(moveit_core REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tmc_robot_kinematics_model REQUIRED)

add_library(${PROJECT_NAME} SHARED src/hsrb_moveit_kinematics.cpp)
ament_target_dependencies(${PROJECT_NAME} hsrb_analytic_ik moveit_core tf2_eigen tmc_robot_kinematics_model)

pluginlib_export_plugin_description_file(moveit_core hsrb_moveit_kinematics_plugin_description.xml)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(tmc_manipulation_tests REQUIRED)

  ament_add_gtest(hsrb_moveit_kinematics_test test/hsrb_moveit_kinematics-test)
  target_link_libraries(hsrb_moveit_kinematics_test ${PROJECT_NAME})
  ament_target_dependencies(hsrb_moveit_kinematics_test tmc_manipulation_tests)
endif()

ament_export_include_directories()
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(hsrb_analytic_ik moveit_core tf2_eigen tmc_robot_kinematics_model)

ament_package()
