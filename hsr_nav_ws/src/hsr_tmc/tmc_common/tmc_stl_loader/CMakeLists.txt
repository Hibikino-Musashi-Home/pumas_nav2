cmake_minimum_required(VERSION 3.8)
project(tmc_stl_loader)

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
find_package(ament_index_cpp REQUIRED)

find_package(Eigen3 REQUIRED)

add_library(stl_loader SHARED src/stl_loader.cpp)
target_include_directories(stl_loader
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
    ${EIGEN3_INCLUDE_DIRS})

ament_target_dependencies(stl_loader ament_index_cpp)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(stl_loader_test test/stl_loader-test.cpp WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
  target_link_libraries(stl_loader_test stl_loader)
endif()

install(TARGETS stl_loader
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)
install(
  DIRECTORY test
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(stl_loader)
ament_export_dependencies(ament_index_cpp)

ament_package()
