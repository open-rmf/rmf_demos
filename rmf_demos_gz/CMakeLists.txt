cmake_minimum_required(VERSION 3.20)

project(rmf_demos_gz)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
