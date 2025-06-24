cmake_minimum_required(VERSION 3.5)
project(tmc_manipulation_tests)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)

add_library(configs SHARED src/configs.cpp)
target_include_directories(configs PUBLIC include)
ament_target_dependencies(configs ament_index_cpp)

install(TARGETS configs
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)
install(
  DIRECTORY meshes robots urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(configs)
ament_export_dependencies(ament_index_cpp)

ament_package()

# This package has no tests
