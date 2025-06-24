cmake_minimum_required(VERSION 3.5)
project(tmc_collision_detector)

find_package(ament_cmake REQUIRED)
find_package(tmc_manipulation_types REQUIRED)
find_package(tmc_stl_loader REQUIRED)

find_package(Eigen3 REQUIRED)

add_library(ODE_collision_detector SHARED src/ODE_collision_detector.cpp)
target_include_directories(ODE_collision_detector PUBLIC include ${EIGEN3_INCLUDE_DIRS})
target_link_libraries(ODE_collision_detector ode)
ament_target_dependencies(ODE_collision_detector tmc_manipulation_types tmc_stl_loader)


if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(geometric_shapes REQUIRED)

  ament_add_gtest(gtest_collision_detector test/gtest_collision_detector.cpp WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
  target_link_libraries(gtest_collision_detector ODE_collision_detector)
  ament_target_dependencies(gtest_collision_detector geometric_shapes)
endif()

install(TARGETS ODE_collision_detector
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(ODE_collision_detector)
ament_export_dependencies(tmc_manipulation_types tmc_stl_loader)

ament_package()
