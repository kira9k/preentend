cmake_minimum_required(VERSION 3.8)
project(preentend_gazebo)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch worlds meshes models DESTINATION share/${PROJECT_NAME})

ament_package()
