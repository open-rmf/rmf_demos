cmake_minimum_required(VERSION 3.5)

project(demos)

find_package(ament_cmake REQUIRED)

install(
  PROGRAMS
    scripts/airport_terminal/airport_terminal_scenario.sh
    scripts/airport_terminal/airport_terminal_spawn_robots.sh
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
