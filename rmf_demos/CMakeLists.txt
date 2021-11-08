cmake_minimum_required(VERSION 3.5)

project(rmf_demos)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch/
  config
  sros2
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
