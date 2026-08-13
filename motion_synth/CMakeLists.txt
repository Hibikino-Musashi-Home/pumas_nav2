cmake_minimum_required(VERSION 3.8)
project(motion_synth)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)

find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(actionlib_msgs REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

find_package(pumas_interfaces REQUIRED)

# C++ section
add_executable(motion_synth_server
    src/motion_synth_server.cpp
)

ament_target_dependencies(motion_synth_server
    rclcpp
    rclcpp_action

    sensor_msgs
    geometry_msgs
    nav_msgs
    actionlib_msgs

    tf2
    tf2_ros

    pumas_interfaces
)

install(TARGETS
  motion_synth_server
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
