cmake_minimum_required(VERSION 3.5)

project(rmf_demos_maps)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  maps/
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

file(GLOB_RECURSE traffic_editor_paths "maps/*.building.yaml")

if (NOT NO_DOWNLOAD_MODELS)
  add_custom_target(
    update_model_cache
    COMMAND ros2 run rmf_building_map_tools building_map_model_downloader
  )
endif()

foreach(path ${traffic_editor_paths})

  # Get the output world name
  string(REGEX REPLACE "\\.[^.]*\.[^.]*$" "" no_extension_path ${path})
  string(REGEX MATCH "[^\/]+$" world_name  ${no_extension_path})

  set(map_path ${path})
  set(output_world_name ${world_name})
  set(output_dir ${CMAKE_CURRENT_BINARY_DIR}/maps/${output_world_name})
  set(output_world_path ${output_dir}/${output_world_name}.world)
  set(output_model_dir ${output_dir}/models)

  ##############################################################################
  # Generate Gz world and download Models
  ##############################################################################

  message("BUILDING WORLDFILE WITH COMMAND: ros2 run rmf_building_map_tools building_map_generator gazebo ${map_path} ${output_world_path} ${output_model_dir}")
  add_custom_target(
    ${world_name}
    COMMAND ros2 run rmf_building_map_tools building_map_generator gazebo ${map_path} ${output_world_path} ${output_model_dir}
  )
  if (NOT NO_DOWNLOAD_MODELS)
    add_dependencies(${world_name} update_model_cache)
  endif()

  ##############################################################################
  # generate the navmesh and required files for crowd simulation for gz
  ##############################################################################
  set(crowd_sim_config_resource ${output_dir}/config_resource/)

  add_custom_command(
    OUTPUT ${world_name}_crowdsim
    COMMAND ros2 run rmf_building_map_tools building_crowdsim ${map_path} ${crowd_sim_config_resource} ${output_world_path}
    DEPENDS ${world_name}
  )

  # This will initiate both custom commands: ${output_world_path} and ${world_name}_crowdsim
  add_custom_target(generate_${world_name}_crowdsim ALL
    DEPENDS ${world_name}_crowdsim
  )

  ##############################################################################
  # Generate the nav graphs
  ##############################################################################

  set(output_nav_graphs_dir ${output_dir}/nav_graphs/)
  set(output_nav_graphs_phony ${output_nav_graphs_dir}/phony)
  add_custom_command(
    OUTPUT ${output_nav_graphs_phony}
    COMMAND ros2 run rmf_building_map_tools building_map_generator nav ${map_path} ${output_nav_graphs_dir}
    DEPENDS ${map_path}
  )

  add_custom_target(generate_${output_world_name}_nav_graphs ALL
    DEPENDS ${output_nav_graphs_phony}
  )

  install(
    DIRECTORY ${output_dir}
    DESTINATION share/${PROJECT_NAME}/maps
  )

endforeach()
