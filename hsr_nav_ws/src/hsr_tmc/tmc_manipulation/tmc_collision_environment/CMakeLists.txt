cmake_minimum_required(VERSION 3.5)
project(tmc_collision_environment)

find_package(ament_cmake REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tmc_utils REQUIRED)
find_package(visualization_msgs REQUIRED)

add_library(collision_environment_server SHARED src/collision_environment_server.cpp src/utils.cpp)
ament_target_dependencies(collision_environment_server
  moveit_msgs
  rclcpp
  rclcpp_components
  tf2_eigen
  tf2_geometry_msgs
  tf2_ros
  tmc_utils
  visualization_msgs
)
rclcpp_components_register_node(collision_environment_server PLUGIN "tmc_collision_environment::CollisionEnvironmentServer" EXECUTABLE collision_environment_node)

add_library(attached_object_server SHARED src/attached_object_publisher.cpp src/utils.cpp)
ament_target_dependencies(attached_object_server
  moveit_msgs
  rclcpp
  rclcpp_components
  tf2_eigen
  tf2_ros
  tmc_utils
  visualization_msgs
)
rclcpp_components_register_node(attached_object_server PLUGIN "tmc_collision_environment::AttachedObjectServer" EXECUTABLE attached_object_publisher)

install(TARGETS collision_environment_server attached_object_server
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

find_package(ament_cmake_python REQUIRED)
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS examples/add_static_collision_object.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_collision_environment test/test_collision_environment.cpp test/utils.cpp)
  target_link_libraries(test_collision_environment collision_environment_server)

  ament_add_gtest(test_attached_object_server test/test_attached_object_publisher.cpp test/utils.cpp)
  target_link_libraries(test_attached_object_server attached_object_server)
endif()

ament_package()
