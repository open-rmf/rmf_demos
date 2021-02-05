cmake_minimum_required(VERSION 3.5)

project(rmf_demos_dashboard_resources LANGUAGES NONE)

find_package(ament_cmake REQUIRED)

ament_package()

set(outputs
  ${CMAKE_CURRENT_SOURCE_DIR}/airport_terminal/main.json
  ${CMAKE_CURRENT_SOURCE_DIR}/office/main.json
)

if(${CMAKE_VERSION} VERSION_GREATER_EQUAL 3.12)
  file(GLOB_RECURSE deps CONFIGURE_DEPENDS *.json)
else()
  file(GLOB_RECURSE deps *.json)
endif()

list(FILTER deps EXCLUDE REGEX main.json$)

add_custom_command(OUTPUT ${outputs}
  COMMAND python3 build_configuration.py
  DEPENDS ${deps} build_configuration.py
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)

add_custom_target(build ALL DEPENDS ${outputs})

install(DIRECTORY
  airport_terminal
  office
  hotel
  clinic
  DESTINATION share/${PROJECT_NAME}
)
