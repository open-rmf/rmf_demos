cmake_minimum_required(VERSION 3.5)

project(rmf_demo_assets)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY models media
  DESTINATION share/${PROJECT_NAME}
)

ament_package()