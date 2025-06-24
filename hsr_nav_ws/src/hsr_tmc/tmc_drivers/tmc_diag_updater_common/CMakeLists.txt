cmake_minimum_required(VERSION 3.5)
project(tmc_diag_updater_common)

find_package(rclcpp REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(diagnostic_updater REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(ament_cmake REQUIRED)

ament_export_dependencies(diagnostic_msgs diagnostic_updater sensor_msgs Boost
                          rclcpp)

ament_export_include_directories(include)

install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION include/${PROJECT_NAME}/)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)

  include_directories(include ${sensor_msgs_INCLUDE_DIRS})
  add_executable(update_node_stub test/update_node_stub.cpp)
  ament_target_dependencies(update_node_stub diagnostic_updater diagnostic_msgs
                            sensor_msgs rclcpp Boost)
  target_link_libraries(update_node_stub ${sensor_msgs_LIBRARIES}
                        ${rclcpp_LIBRARIES} ${boost_LIBRARIES})

  ament_add_gtest_executable(diag_updater_node_test
                             test/diag_updater_node_test.cpp)
  ament_target_dependencies(diag_updater_node_test diagnostic_updater
                            diagnostic_msgs sensor_msgs rclcpp Boost)

  target_link_libraries(diag_updater_node_test ${sensor_msgs_LIBRARIES}
                        ${rclcpp_LIBRARIES} ${boost_LIBRARIES})
  ament_add_pytest_test(diag_updater_node_test_py
                        "test/diag_updater_node_test.py")

  install(
    TARGETS diag_updater_node_test
    RUNTIME DESTINATION bin
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib)
  install(
    TARGETS update_node_stub
    RUNTIME DESTINATION lib/${PROJECT_NAME}
    ARCHIVE DESTINATION lib/${PROJECT_NAME}
    LIBRARY DESTINATION lib/${PROJECT_NAME})

endif()

ament_package()

