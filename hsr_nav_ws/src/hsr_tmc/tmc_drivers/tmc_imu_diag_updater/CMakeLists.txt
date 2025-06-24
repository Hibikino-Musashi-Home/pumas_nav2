cmake_minimum_required(VERSION 3.5)
project(tmc_imu_diag_updater)

find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tmc_diag_updater_common REQUIRED)
find_package(ament_cmake REQUIRED)
  
ament_export_dependencies(geometry_msgs 
                          rclcpp 
                          sensor_msgs 
                          Boost
                          std_srvs
                          tmc_diag_updater_common)

ament_export_include_directories(include)

include_directories(
  ${rclcpp_INCLUDE_DIRS}
)

# executable
add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}/diag_updater_node.cpp)
target_link_libraries(${PROJECT_NAME}_node
  ${rclcpp_LIBRARIES}
)

ament_target_dependencies(  
  ${PROJECT_NAME}_node
  rclcpp
  sensor_msgs 
  tmc_diag_updater_common
  std_srvs
  Boost)


# install
install(
  TARGETS
    ${PROJECT_NAME}_node
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY
    launch
  DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)
if (BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)

  include_directories(diagnostic_msgs)
  function(add_verifier_test verifier_name timeout)
    ament_add_gtest_executable(${verifier_name}_test
      test/${verifier_name}_test.cpp
    )
    target_link_libraries(${verifier_name}_test
      ${rclcpp_LIBRARIES}
    )
    ament_target_dependencies(  
      ${verifier_name}_test
      rclcpp
      sensor_msgs 
      tmc_diag_updater_common
      std_srvs
      diagnostic_msgs
      Boost)
    install(
      TARGETS ${verifier_name}_test
      RUNTIME DESTINATION bin
      ARCHIVE DESTINATION lib
      LIBRARY DESTINATION lib
    )
    ament_add_pytest_test(${verifier_name}_test_py
                          test/${verifier_name}_test.py
                          TIMEOUT ${timeout} )


  endfunction(add_verifier_test)

  add_verifier_test(contiguous_same_value 60)
  add_verifier_test(zero_velocity_and_acceleration 60)
  add_verifier_test(initial_acceleration_norm 150)
endif()
ament_package()

