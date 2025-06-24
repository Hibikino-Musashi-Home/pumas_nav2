cmake_minimum_required(VERSION 3.5)
project(hsrb_analytic_ik)

find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tmc_robot_kinematics_model REQUIRED)
find_package(Eigen3 REQUIRED)

# Declare a cpp library
add_library(${PROJECT_NAME} SHARED
  src/${PROJECT_NAME}/hsr_common_ik_solver.cpp
  src/${PROJECT_NAME}/hsrb_ik_plugins.cpp
  src/${PROJECT_NAME}/hsrb_ik_solver.cpp
  src/${PROJECT_NAME}/hsrc_ik_solver.cpp
  src/${PROJECT_NAME}/ik_solver_base.cpp
)
set_property(TARGET ${PROJECT_NAME} PROPERTY POSITION_INDEPENDENT_CODE ON)
target_include_directories(${PROJECT_NAME} PUBLIC include ${EIGEN3_INCLUDE_DIRS})
ament_target_dependencies(${PROJECT_NAME} pluginlib tmc_robot_kinematics_model)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(tmc_manipulation_tests REQUIRED)

  # integration testing
  add_library(test_common test/test_common.cpp)
  target_link_libraries(test_common ${PROJECT_NAME})

  ament_add_gtest(test_ik_solver test/ik_solver-test.cpp TIMEOUT 120)
  target_link_libraries(test_ik_solver test_common)
  ament_target_dependencies(test_ik_solver tmc_manipulation_tests)

  # unit testing
  ament_add_gtest(vector2_test test/vector2-test.cpp)
  target_include_directories(vector2_test PRIVATE include src/${PROJECT_NAME})

  ament_add_gtest(matrix2_test test/matrix2-test.cpp)
  target_include_directories(matrix2_test PRIVATE include src/${PROJECT_NAME})

  ament_add_gtest(function_adapters_test test/function_adapters-test.cpp)
  target_include_directories(function_adapters_test PRIVATE include src/${PROJECT_NAME})

  ament_add_gtest(robot_function2 test/robot_function2-test.cpp)
  target_include_directories(robot_function2 PRIVATE include src/${PROJECT_NAME} ${EIGEN3_INCLUDE_DIRS})
  target_link_libraries(robot_function2 ${PROJECT_NAME})
  ament_target_dependencies(robot_function2 tmc_robot_kinematics_model)

  ament_add_gtest(hooke_and_jeeves_method2_test test/hooke_and_jeeves_method2-test.cpp)
  target_include_directories(hooke_and_jeeves_method2_test PRIVATE include src/${PROJECT_NAME})

  ament_add_gtest(bi_golden_section_line_search_test test/bi_golden_section_line_search-test.cpp)
  target_include_directories(bi_golden_section_line_search_test PRIVATE include src/${PROJECT_NAME})

  ament_add_gtest(golden_section_line_search_test test/golden_section_line_search-test.cpp)
  target_include_directories(golden_section_line_search_test PRIVATE include src/${PROJECT_NAME})
endif()

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)

pluginlib_export_plugin_description_file(tmc_robot_kinematics_model hsrb_ik_plugins.xml)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(pluginlib tmc_robot_kinematics_model)

ament_package()
