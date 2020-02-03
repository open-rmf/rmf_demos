cmake_minimum_required(VERSION 3.5)

project(rmf_demo_maps)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  maps/
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
