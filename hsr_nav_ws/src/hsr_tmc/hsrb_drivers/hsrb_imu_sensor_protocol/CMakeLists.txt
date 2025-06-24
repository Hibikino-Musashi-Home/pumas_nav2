cmake_minimum_required(VERSION 3.5)
project(hsrb_imu_sensor_protocol)

find_package(ament_cmake REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(Boost REQUIRED COMPONENTS system)
find_package(rclcpp REQUIRED)

include_directories(include ${Boost_INCLUDE_DIRS})

add_library(
  ${PROJECT_NAME}
  src/${PROJECT_NAME}/mpu9150_packet_parser.cpp
  src/${PROJECT_NAME}/mpu9150_network.cpp
  src/${PROJECT_NAME}/mpu9150_protocol.cpp)

ament_target_dependencies(  
  ${PROJECT_NAME}
  rclcpp
  Boost)

target_link_libraries(  ${PROJECT_NAME}
  ${Boost_LIBRARIES})


# executable
add_executable(mpu9150_node src/${PROJECT_NAME}/mpu9150_node.cpp)
target_link_libraries(mpu9150_node
  ${PROJECT_NAME}
  ${Boost_LIBRARIES}
  ${rclcpp_LIBRARIES}
)

ament_target_dependencies(  
  mpu9150_node
  sensor_msgs
  geometry_msgs
  rclcpp
  std_srvs
  Boost)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include)

install(TARGETS 
    ${PROJECT_NAME}
    mpu9150_node
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY
    launch
  DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_gtest(mpu9150_packet_parser_test test/mpu9150_packet_parser_test.cpp)
  target_link_libraries(mpu9150_packet_parser_test ${PROJECT_NAME})

  ament_add_gtest_executable(mpu9150_protocol_test test/mpu9150_protocol_test.cpp) 
  target_link_libraries(mpu9150_protocol_test ${boost_LIBRARIES} ${PROJECT_NAME})

  ament_add_gtest_executable(mpu9150_node_test test/mpu9150_node_test.cpp) 
  ament_target_dependencies(  
    mpu9150_node_test
    sensor_msgs
    geometry_msgs
    rclcpp
    std_srvs
    Boost)

  target_link_libraries(mpu9150_node_test ${boost_LIBRARIES} ${PROJECT_NAME})

  ament_add_pytest_test(mpu9150_protocol_test_py "test/mpu9150_protocol_test.py"
    PYTHON_EXECUTABLE "${_PYTHON_EXECUTABLE}"
    WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}"
  )
  ament_add_pytest_test(mpu9150_node_test_py "test/mpu9150_node_test.py"
    PYTHON_EXECUTABLE "${_PYTHON_EXECUTABLE}"
    WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}"
  )

  install(
    TARGETS
    mpu9150_protocol_test
    mpu9150_node_test
    RUNTIME DESTINATION bin
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
  )

endif()

ament_package()

