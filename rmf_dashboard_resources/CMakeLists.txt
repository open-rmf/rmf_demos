cmake_minimum_required(VERSION 3.5)

project(rmf_demos_dashboard_resources LANGUAGES NONE)

find_package(ament_cmake REQUIRED)

ament_package()

add_custom_target(build ALL
  COMMAND python3 build_configuration.py
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)

install(DIRECTORY
  airport_terminal
  office
  DESTINATION share/${PROJECT_NAME}
)
