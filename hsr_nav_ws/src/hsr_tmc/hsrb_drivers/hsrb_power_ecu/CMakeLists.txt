cmake_minimum_required(VERSION 3.5)
project(hsrb_power_ecu)

find_package(ament_cmake REQUIRED)
find_package(Boost REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/power_ecu_com_data_decoder.cpp
  src/power_ecu_com_data_encoder.cpp
  src/power_ecu_com_frame_decoder.cpp
  src/power_ecu_com_frame_encoder.cpp
  src/power_ecu_com_common.cpp
  src/power_ecu_protocol.cpp
  src/serial_network.cpp
  src/power_ecu_com_repro_data_encoder.cpp
  src/battery_state_publisher.cpp
  src/bool_state_publisher.cpp
  src/imu_state_publisher.cpp
  src/runstop_state_publisher.cpp
  src/led_command_subscriber.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC include)
ament_target_dependencies(${PROJECT_NAME} rclcpp std_msgs sensor_msgs Boost)

add_executable(${PROJECT_NAME}_main src/main_loop.cpp)
ament_target_dependencies(${PROJECT_NAME}_main std_msgs sensor_msgs rclcpp Boost)
target_include_directories(${PROJECT_NAME}_main PUBLIC include)
target_link_libraries(${PROJECT_NAME}_main ${PROJECT_NAME})

add_executable(info src/info_command.cpp)
ament_target_dependencies(info rclcpp Boost)
target_include_directories(info PUBLIC include)
target_link_libraries(info ${PROJECT_NAME})

add_executable(battery src/example_battery_state_publisher.cpp)
ament_target_dependencies(battery sensor_msgs rclcpp Boost)
target_include_directories(battery PUBLIC include)
target_link_libraries(battery ${PROJECT_NAME})

add_executable(bools src/example_bool_state_publisher.cpp)
ament_target_dependencies(bools std_msgs rclcpp Boost)
target_include_directories(bools PUBLIC include)
target_link_libraries(bools ${PROJECT_NAME})

add_executable(imu src/example_imu_state_publisher.cpp)
ament_target_dependencies(imu sensor_msgs rclcpp Boost)
target_include_directories(imu PUBLIC include)
target_link_libraries(imu ${PROJECT_NAME})

add_executable(led src/example_led_command_subscriber.cpp)
ament_target_dependencies(led std_msgs rclcpp Boost)
target_include_directories(led PUBLIC include)
target_link_libraries(led ${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_main info battery bools imu led
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include
)
install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)

  add_library(${PROJECT_NAME}_test SHARED test/network_mock.cpp)
  target_include_directories(${PROJECT_NAME}_test PUBLIC include)
  ament_target_dependencies(${PROJECT_NAME}_test rclcpp)

  ament_add_gtest(main_loop_test test/main_loop-test.cpp)
  target_link_libraries(main_loop_test ${PROJECT_NAME} ${PROJECT_NAME}_test)

  ament_add_gtest(test_error_counter test/error_counter-test.cpp)
  target_link_libraries(test_error_counter ${PROJECT_NAME})

  ament_add_gtest(test_power_ecu_com_common test/power_ecu_com_common-test.cpp)
  target_link_libraries(test_power_ecu_com_common ${PROJECT_NAME})

  ament_add_gtest(test_power_ecu_com_element_encoder test/power_ecu_com_element_encoder-test.cpp)
  target_link_libraries(test_power_ecu_com_element_encoder ${PROJECT_NAME})

  ament_add_gtest(test_power_ecu_com_element_decoder test/power_ecu_com_element_decoder-test.cpp)
  target_link_libraries(test_power_ecu_com_element_decoder ${PROJECT_NAME})

  ament_add_gtest(test_power_ecu_com_data_encoder test/power_ecu_com_data_encoder-test.cpp)
  target_link_libraries(test_power_ecu_com_data_encoder ${PROJECT_NAME})

  ament_add_gtest(test_power_ecu_com_data_decoder test/power_ecu_com_data_decoder-test.cpp)
  target_link_libraries(test_power_ecu_com_data_decoder ${PROJECT_NAME})

  ament_add_gtest(test_power_ecu_com_frame_encoder test/power_ecu_com_frame_encoder-test.cpp)
  target_link_libraries(test_power_ecu_com_frame_encoder ${PROJECT_NAME})

  ament_add_gtest(test_power_ecu_com_frame_decoder test/power_ecu_com_frame_decoder-test.cpp)
  target_link_libraries(test_power_ecu_com_frame_decoder ${PROJECT_NAME})

  ament_add_gmock(test_serial_network test/serial_network-test.cpp)
  target_link_libraries(test_serial_network ${PROJECT_NAME})
endif()

ament_export_dependencies(rclcpp std_msgs sensor_msgs)
ament_package()
