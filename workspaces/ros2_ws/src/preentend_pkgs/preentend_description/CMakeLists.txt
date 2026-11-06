cmake_minimum_required(VERSION 3.8)
project(preentend_description)

# find dependencies
find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch robot urdf meshes rviz
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
