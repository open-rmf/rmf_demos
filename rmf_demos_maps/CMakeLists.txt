cmake_minimum_required(VERSION 3.5)

project(rmf_demos_maps)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  maps/
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

file(GLOB_RECURSE traffic_editor_paths "maps/*.site.json")

foreach(path ${traffic_editor_paths})

  # Get the output world name
  string(REGEX REPLACE "\\.[^.]*\.[^.]*$" "" no_extension_path ${path})
  string(REGEX MATCH "[^\/]+$" world_name  ${no_extension_path})

  set(map_path ${path})
  set(output_world_name ${world_name})
  set(output_dir ${CMAKE_CURRENT_BINARY_DIR}/maps/${output_world_name})
  set(output_world_path ${output_dir}/${output_world_name}.world)

  ##############################################################################
  # Generate Gz world and download Models
  ##############################################################################

  message("BUILDING WORLDFILE WITH COMMAND: ros2 run rmf_site_editor rmf_site_editor ${map_path} --headless-export ${output_world_path}")
  if (NO_DOWNLOAD_MODELS)
    # NOT IMPLEMENTED
  else()
    add_custom_command(
      DEPENDS ${map_path}
      COMMAND ros2 run rmf_site_editor rmf_site_editor ${map_path} --headless-export ${output_world_path}
      #COMMAND ros2 run rmf_building_map_tools building_map_generator gazebo ${map_path} ${output_world_path} ${output_model_dir}
      #COMMAND ros2 run rmf_building_map_tools building_map_model_downloader ${map_path} -e ~/.gazebo/models
      OUTPUT ${output_world_path}
    )
  endif()

  ##############################################################################
  # generate the navmesh and required files for crowd simulation for gz
  ##############################################################################
  set(crowd_sim_config_resource ${output_dir}/config_resource/)

  add_custom_command(
    OUTPUT ${world_name}_crowdsim
    COMMAND ros2 run rmf_building_map_tools building_crowdsim ${map_path} ${crowd_sim_config_resource} ${output_world_path}
    DEPENDS ${output_world_path}
  )

  # This will initiate both custom commands: ${output_world_path} and ${world_name}_crowdsim
  add_custom_target(generate_${world_name}_crowdsim ALL
    DEPENDS ${world_name}_crowdsim
  )

  install(
    DIRECTORY ${output_dir}
    DESTINATION share/${PROJECT_NAME}/maps
  )

endforeach()
