cmake_minimum_required(VERSION 3.5)
project(tmc_robot_collision_detector)

find_package(ament_cmake REQUIRED)
find_package(tmc_collision_detector REQUIRED)
find_package(tmc_manipulation_types REQUIRED)
find_package(tmc_robot_kinematics_model REQUIRED)
find_package(urdf REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(PkgConfig)

find_package(tinyxml_vendor REQUIRED)
find_package(TinyXML REQUIRED)

add_library(collision_detector_config SHARED src/collision_detector_config.cpp)
target_include_directories(collision_detector_config PUBLIC include)
ament_target_dependencies(collision_detector_config TinyXML)

add_library(collision_detector_factory SHARED src/collision_detector_factory.cpp)
target_include_directories(collision_detector_factory PUBLIC ${EIGEN3_INCLUDE_DIRS})
ament_target_dependencies(collision_detector_factory tmc_collision_detector)

add_library(robot_collision_detector SHARED src/robot_collision_detector.cpp)
target_include_directories(robot_collision_detector PUBLIC include ${EIGEN3_INCLUDE_DIRS})
ament_target_dependencies(robot_collision_detector tmc_collision_detector tmc_manipulation_types tmc_robot_kinematics_model urdf)
target_link_libraries(robot_collision_detector collision_detector_config collision_detector_factory)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(tmc_manipulation_tests REQUIRED)

  ament_add_gtest(test_collision_detector_config test/collision_detector_config-test.cpp
                  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
  target_link_libraries(test_collision_detector_config collision_detector_config)

  ament_add_gtest(test_robot_collision_detector test/robot_collision_detector-test.cpp
                  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
  target_link_libraries(test_robot_collision_detector robot_collision_detector)
ament_target_dependencies(test_robot_collision_detector tmc_manipulation_tests)
endif()

install(TARGETS collision_detector_config collision_detector_factory robot_collision_detector
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(robot_collision_detector)
ament_export_dependencies(tinyxml_vendor TinyXML tmc_collision_detector tmc_manipulation_types tmc_robot_kinematics_model urdf)

ament_package()
