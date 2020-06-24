^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package demos
^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------

1.0.0 (2020-06-24)
------------------
* Develop (`#85 <https://github.com/osrf/rmf_demos/issues/85>`_)
  * Updated request_delivery with pickup and dropoff dispener names
  * Fixed dangling waypoint
  * Added newly migrated Door panels
  * Reverted some automatic changes when updating configs
  * Adding marker subscriber for /building_systems_markers
  * Fixes for first release (`#84 <https://github.com/osrf/rmf_demos/issues/84>`_)
  * Removed workcell names from maps and added delivery launch files
  * Updated RMF panel with new Delivery message structure. Updated rviz configs
  * Linted with pycodestyle
  * Updated build.yaml
  * RMF linter
  * Fixed EOL error
  * Updated README.md
  Co-authored-by: Yadunund <yadunund@openrobotics.org>
* Merge pull request `#75 <https://github.com/osrf/rmf_demos/issues/75>`_ from osrf/develop
  Develop
* Moved read_only_fleet_adapter launch into airport_terminal.launch.xml
* Updated mir vicinity
* Updated robot vicinity radii
* Updated adapter launches
* Updated API call usage in RMFPanel
* Merge remote-tracking branch 'origin/master' into new_interface
* Merge pull request `#68 <https://github.com/osrf/rmf_demos/issues/68>`_ from osrf/fix/airport_scenarios
  Fix/airport scenarios
* Updated README.md
* Cleaned up scenarios
* Merge remote-tracking branch 'origin/master' into develop_compability
* Merge branch 'fix/airport_scenarios' of github.com:osrf/rmf_demos into fix/airport_scenarios
* Updated scenarios
* Merge branch 'master' into fix/airport_scenarios
* fix gazebo_ros_pkgs meta package as dependency (`#65 <https://github.com/osrf/rmf_demos/issues/65>`_)
* Added loop launch file
* Merge pull request `#63 <https://github.com/osrf/rmf_demos/issues/63>`_ from osrf/feature/airport-dispenser
  added new respawn seconds sdf element, added delivery scenario
* added new respawn seconds sdf element, added delivery scenario
* Feature/teleport plugin v2 (`#62 <https://github.com/osrf/rmf_demos/issues/62>`_)
  * experimenting with placeholder links on models, refactoring teleport to have dispenser stuff
  * no need for mutexes
  * fixed RobotPlaceholder model, teleport plugin works, now v3 to remove target name
  * extended respawn to 5 seconds, caching modelptrs instead of names
  * handle duplicate dispenser request guids
  * cleaned up dispenser script and usage
  * removed launch call for dispensers
* Disabled caddy merge lane
* Modified motion params of Mir100
* Matched magni fleet adapter parameters with those in slotcar
* Modified caddy velocity parameters
* Added more doors
* Modified magni_2 placeholder
* Modified caddy placeholder
* Updated scenarios
* Updated scenarios
* Merge pull request `#56 <https://github.com/osrf/rmf_demos/issues/56>`_ from osrf/fix/launch-xml
  added launch xml to exec depend
* added launch xml to exec depend
* Minor enhancement to readonly plugin (`#52 <https://github.com/osrf/rmf_demos/issues/52>`_)
  * POC merge lane
  * POC merge lane
  * Added lane_threshold to control merging of robot into its lane
  * Spawning caddy without launching airport_terminal world
  * Format fix
* Feature/split scenarios (`#48 <https://github.com/osrf/rmf_demos/issues/48>`_)
  * split launches into caddy and no-caddy, fixed missing mir100_2
  * fix overlapping lanes, door remains open issue
  * fixed another overlap, door waypoints slightly further
* Merge branch 'master' of https://github.com/osrf/rmf_demos
* Feature/doors (`#42 <https://github.com/osrf/rmf_demos/issues/42>`_)
  * added double hinged doors at strategic locations
  * testing double hinged door on office, adding new robot behind doors scenario
  * corrected office building test
  * added lanes, changed 3rd mir loop route to pass through doors, manually added wall segments for doors
  * Ensure lanes overlap
  * Added two new loop scenarios
  * Install loop scenario files
  * Modified position of junction_n02
  Co-authored-by: Yadunund <yadunund@openrobotics.org>
* Merge pull request `#32 <https://github.com/osrf/rmf_demos/issues/32>`_ from osrf/fix/rviz_configs
  Updated configs
* Feature/office single doors (`#31 <https://github.com/osrf/rmf_demos/issues/31>`_)
  * using building_gazebo_plugins instead, cleaned up, WIP testing hinged doors
  * removed doors in airport first
* Updated configs
* Merge pull request `#26 <https://github.com/osrf/rmf_demos/issues/26>`_ from osrf/feature/caddy
  Feature/caddy
* Fixes
* merge conflict
* Fixes
* Merge pull request `#25 <https://github.com/osrf/rmf_demos/issues/25>`_ from osrf/feature/spawn-robots
  Feature/spawn robots
* removed unused launch files, removed race-condition task request launch file
* adding second script to only spawn without sending any tasks
* merged master, deconflicted
* change to using shell script to prevent race condition, TODO check delays on laptops
* Cleanup
* scenario launch file and delayed request
* starting work on delayed loop requester
* Modified caddy drive parameters and introduced rmf_traffic dependency
* Teleop joystick added to launch
* renamed the launch files to corrent ones, WIP hesitant robots
* added spawn and loop launch scripts, WIP resolving issue of hesitant robots
* spawn robots instead of adding to sdf, WIP issues with motion after spawning
* Initialize start waypoint
* Feature/airport terminal (`#24 <https://github.com/osrf/rmf_demos/issues/24>`_)
  * added basic airport terminal demo
  * added screenshot
  * fixed office launch missing arg, added more loops in launch, cleaned up waypoints, added new fleet lanes
  * organized launch files, fixed nav graph number, lane mistakes
  * fixed duplicate waypoint name
  * fixed launch file name
  * spawning magnis outside of charging room to save some time on getting out
  * adding loop launch instruction to readme
  * Fixed argument in office launch and map_name in rviz panel
  Co-authored-by: Yadunund <yadunund@openrobotics.org>
* Added caddy model with readonly plugin
* Fixed argument in office launch and map_name in rviz panel
* spawning magnis outside of charging room to save some time on getting out
* fixed launch file name
* organized launch files, fixed nav graph number, lane mistakes
* fixed office launch missing arg, added more loops in launch, cleaned up waypoints, added new fleet lanes
* added basic airport terminal demo
* Merge pull request `#22 <https://github.com/osrf/rmf_demos/issues/22>`_ from osrf/fix/common-viz-var
  Fix/common viz var
* changing launch xml files to use the var
* Merge pull request `#17 <https://github.com/osrf/rmf_demos/issues/17>`_ from osrf/rviz_plugin
  Rviz plugin
* Updated office.rviz
* Fixed merge conflict
* Fixed dispenser name
* Change .traffic-editor. to .building.
* Merge pull request `#9 <https://github.com/osrf/rmf_demos/issues/9>`_ from osrf/upgrade_editor_file_format
  tweak building filename suffix and add editor project file
* Added beverage collector dispenser
* Enabled magni fleet adapter to perform deliveries
* Added dispenser node
* Added delivery request task
* tweak building filename suffix and add editor project file
* Added office_loop launch
* Added door supervisor
* Passing office nav graph to magni fleet adapter
* Merge pull request `#8 <https://github.com/osrf/rmf_demos/issues/8>`_ from osrf/feature/add_world
  Feature/add world
* Added lanes
* Formatting Fixes
* Fixed media and updated office editor
* Added assets for demos
* Added rmf_demo_assets
* Modified launch
* Added launch file to bringup office world
* Contributors: Aaron, Aaron Chong, Boon Han, Charayaphan Nakorn Boon Han, Morgan Quigley, Yadu, Yadunund
