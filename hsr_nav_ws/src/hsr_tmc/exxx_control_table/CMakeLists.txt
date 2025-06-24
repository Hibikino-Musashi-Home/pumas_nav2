# This package has no tests
cmake_minimum_required(VERSION 3.5)
project(exxx_control_table)
find_package(ament_cmake REQUIRED)

ament_export_dependencies()

install(DIRECTORY control_tables/
        DESTINATION share/exxx_control_table/control_tables
)

ament_package()