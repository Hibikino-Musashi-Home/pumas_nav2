cmake_minimum_required(VERSION 3.5)
project(hsrb_meshes)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY meshes
  DESTINATION share/${PROJECT_NAME}
)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.dsv.in")

ament_package()
