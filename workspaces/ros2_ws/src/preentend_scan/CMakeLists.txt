cmake_minimum_required(VERSION 3.22)
project(preentend_scan)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/scan_console.py DESTINATION lib/${PROJECT_NAME})

ament_package()
