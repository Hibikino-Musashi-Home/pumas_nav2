cmake_minimum_required(VERSION 3.5)
project(hsrb_servomotor_protocol)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)

find_package(Boost REQUIRED COMPONENTS system filesystem)

add_library(${PROJECT_NAME} SHARED
  src/${PROJECT_NAME}/exxx_error_category.cpp
  src/${PROJECT_NAME}/exxx_warning_category.cpp
  src/${PROJECT_NAME}/exxx_network.cpp
  src/${PROJECT_NAME}/exxx_protocol.cpp
  src/${PROJECT_NAME}/exxx_reprograming.cpp
  src/${PROJECT_NAME}/control_table.cpp
  src/${PROJECT_NAME}/control_table_item_descriptor.cpp
  src/${PROJECT_NAME}/load_control_table.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC include)
ament_target_dependencies(${PROJECT_NAME} rclcpp)
target_link_libraries(${PROJECT_NAME} crypto ${Boost_LIBRARIES})

add_executable(exxx_write_data src/${PROJECT_NAME}/write_data.cpp)
target_include_directories(exxx_write_data PRIVATE include)
target_link_libraries(exxx_write_data ${PROJECT_NAME})

add_executable(exxx_read_data src/${PROJECT_NAME}/read_data.cpp)
target_include_directories(exxx_read_data PRIVATE include)
target_link_libraries(exxx_read_data ${PROJECT_NAME})

add_executable(exxx_read_hash src/${PROJECT_NAME}/read_hash.cpp)
target_include_directories(exxx_read_hash PRIVATE include)
target_link_libraries(exxx_read_hash ${PROJECT_NAME})

add_executable(exxx_write_eeprom src/${PROJECT_NAME}/write_eeprom.cpp)
target_include_directories(exxx_write_eeprom PRIVATE include)
target_link_libraries(exxx_write_eeprom ${PROJECT_NAME})

add_executable(exxx_read_data_table src/${PROJECT_NAME}/read_data_table.cpp)
target_include_directories(exxx_read_data_table PRIVATE include)
ament_target_dependencies(exxx_read_data_table rclcpp ament_index_cpp)
target_link_libraries(exxx_read_data_table ${PROJECT_NAME})

add_executable(exxx_write_data_table src/${PROJECT_NAME}/write_data_table.cpp)
target_include_directories(exxx_write_data_table PRIVATE include)
ament_target_dependencies(exxx_write_data_table rclcpp ament_index_cpp)
target_link_libraries(exxx_write_data_table ${PROJECT_NAME})

add_executable(exxx_flush src/${PROJECT_NAME}/flush.cpp)
target_include_directories(exxx_flush PRIVATE include)
target_link_libraries(exxx_flush ${PROJECT_NAME})

add_executable(check_control_table src/${PROJECT_NAME}/check_control_table.cpp)
target_include_directories(check_control_table PRIVATE include)
target_link_libraries(check_control_table ${PROJECT_NAME})

ament_environment_hooks(${CMAKE_CURRENT_SOURCE_DIR}/scripts/exxx_bash_completion.bash)

install(TARGETS
  ${PROJECT_NAME}
  exxx_write_data
  exxx_read_data
  exxx_read_hash
  exxx_write_eeprom
  exxx_read_data_table
  exxx_write_data_table
  exxx_flush
  check_control_table
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(packet_parser-test test/packet_parser-test.cpp)
  target_include_directories(${PROJECT_NAME} PRIVATE include)
  target_link_libraries(packet_parser-test ${PROJECT_NAME})

  ament_add_gmock(exxx_protocol-test test/exxx_protocol-test.cpp)
  target_include_directories(${PROJECT_NAME} PRIVATE include)
  target_link_libraries(exxx_protocol-test ${PROJECT_NAME})

  ament_add_gtest(control_table-test test/control_table-test.cpp WORKING_DIRECTORY ${PROJECT_SOURCE_DIR})
  target_include_directories(${PROJECT_NAME} PRIVATE include)
  target_link_libraries(control_table-test ${PROJECT_NAME})

  ament_add_gtest(load_control_table-test test/load_control_table-test.cpp WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
  target_include_directories(${PROJECT_NAME} PRIVATE include)
  target_link_libraries(load_control_table-test ${PROJECT_NAME})
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(rclcpp)

ament_package()
