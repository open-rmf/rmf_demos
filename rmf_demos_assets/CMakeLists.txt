cmake_minimum_required(VERSION 3.5)

project(rmf_demos_assets)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY models
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
