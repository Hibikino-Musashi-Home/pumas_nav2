cmake_minimum_required(VERSION 3.8)
project(tmc_utils)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

add_library(${PROJECT_NAME} SHARED src/msg_io.cpp)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

ament_target_dependencies(${PROJECT_NAME} rclcpp)

install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(action_tutorials_interfaces REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(example_interfaces REQUIRED)
  find_package(std_msgs REQUIRED)

  find_package(Boost REQUIRED COMPONENTS filesystem)

  ament_add_gtest(test_caching_subscriber test/caching_subscriber-test.cpp)
  target_include_directories(test_caching_subscriber
    PRIVATE
      "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
      "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
  ament_target_dependencies(test_caching_subscriber rclcpp std_msgs)

  ament_add_gtest(test_msg_io test/msg_io-test.cpp)
  target_include_directories(test_msg_io
    PRIVATE
      "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
      "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
      ${Boost_INCLUDE_DIRS})
  target_link_libraries(test_msg_io ${PROJECT_NAME} ${Boost_LIBRARIES})
  ament_target_dependencies(test_msg_io action_tutorials_interfaces example_interfaces rclcpp std_msgs)

  ament_add_gtest(test_parameters test/parameters-test.cpp)
  target_include_directories(test_parameters
    PRIVATE
      "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
      "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
  ament_target_dependencies(test_parameters rclcpp std_msgs)
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(rclcpp)

ament_package()
