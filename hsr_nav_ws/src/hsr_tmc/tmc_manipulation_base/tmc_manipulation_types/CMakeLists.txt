cmake_minimum_required(VERSION 3.5)
project(tmc_manipulation_types)

find_package(ament_cmake REQUIRED)

find_package(Eigen3 REQUIRED)

add_library(${PROJECT_NAME} src/${PROJECT_NAME}/utils.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC include ${EIGEN3_INCLUDE_DIRS})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_manipulaton_types test/manipulation_types-test.cpp)
  target_include_directories(test_manipulaton_types PRIVATE include ${EIGEN3_INCLUDE_DIRS})

  ament_add_gtest(test_utils test/utils-test.cpp)
  target_link_libraries(test_utils ${PROJECT_NAME})
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(Eigen3)

ament_package()