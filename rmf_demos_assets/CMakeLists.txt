cmake_minimum_required(VERSION 3.5)

project(rmf_demos_assets)

find_package(ament_cmake REQUIRED)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")

install(
  DIRECTORY models
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
