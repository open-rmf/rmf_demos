^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rmf_demo_maps
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* Using new building_map_generator (`#77 <https://github.com/osrf/rmf_demos/issues/77>`_)
  * Using new building_map_generator
  * Added instructions for new python dependency from pit_crew, added to workflow
* Started using new commands from building_map_tools (`#76 <https://github.com/osrf/rmf_demos/issues/76>`_)
* Feature/non intrusive dispenser (`#69 <https://github.com/osrf/rmf_demos/issues/69>`_)
  * finished first iteration of TeleportDispenser, crashing at world start
  * ingestor working, need to figure out how to teleport it back to dispenser
  * TeleportIngestor working with office and airport demos, removed old teleport plugin
  * removed old teleport plugin, removed odd vscode artifact
  * minor cleanup, const correctness
  * removed delay before dispensing, made respawn by ingestor 10 seconds
  * made function names more verbose and const qualify variables
* Merge remote-tracking branch 'origin/master' into new_interface
* Merge pull request `#68 <https://github.com/osrf/rmf_demos/issues/68>`_ from osrf/fix/airport_scenarios
  Fix/airport scenarios
* Merge remote-tracking branch 'origin/master' into develop_compability
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
* Removed oak tree
* Removed oak tree in the middle
* Deleted one door
* Added more doors
* Modified magni_2 placeholder
* Modified caddy placeholder
* Removed magni_1 placeholder
* Added new magni lane
* Changed location of magni_0 placeholder
* Added new magni lane
* Updated scenarios
* Updated scenarios
* Fix/airport map (`#59 <https://github.com/osrf/rmf_demos/issues/59>`_)
  * Downsampled assets in simulation
  * More downsampling
  * Adjusted lanes
  * Adjusted lanes
  * Minor fixes
* Merge pull request `#58 <https://github.com/osrf/rmf_demos/issues/58>`_ from ngwk1/ngwk1-Populate-airport-world
  Populated airport terminal world
* Update airport_terminal.building.yaml
* missed models from dropbox, accidentally lefleft in debugging .building.yaml (`#54 <https://github.com/osrf/rmf_demos/issues/54>`_)
* Feature/model handling (`#53 <https://github.com/osrf/rmf_demos/issues/53>`_)
  * added scripts and basic config file to download fuel models
  * removed fuel models, added instructions to download to disk
  * steps to download fuel models to tmp folders raw
  * updated readme
  * removed office chairs
  * airport_terminal with all the just added assets
  * cleaned up airport_terminal, added fuel_model path to office, included back non-broken officechairs
  * added all the supposed fue fuel models into local repo, will handle downloading from fuel in next features
  * removed model thumbnails and removed ignition fuel installation line on readme
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
* Merge pull request `#37 <https://github.com/osrf/rmf_demos/issues/37>`_ from osrf/fix/static_coke
  Set coke model to non-static
* Set coke model to non-static
* Feature/office single doors (`#31 <https://github.com/osrf/rmf_demos/issues/31>`_)
  * using building_gazebo_plugins instead, cleaned up, WIP testing hinged doors
  * removed doors in airport first
* Feature/caddy track (`#27 <https://github.com/osrf/rmf_demos/issues/27>`_)
  * reduced shop size, reduced number of floor models
  * made the stall islands smaller, WIP adding caddy lanes
  * tidied up the floor tiles, chose texture, added caddy track
  * cleaned up world, removed doors, WIP figure out why doors are not rendering
  * allow plugin to handle single doors, testing .building.yaml
  * testing minor door stuff
  * removed door testing stuff, ready to cleanup and merge
  * cleanup, removed door testing stuff
  * changing the screenshots
  * renewed docs
  * change build test to use rosdep
  * corrected build test
  * rosdep update
  * skipping some keys
  * skipping libgazebo9 in rosdep
  * adding sudo to rosdep command, removing skip keys
  * added skip keys
  * continue installing despite errors
  * removed gazebo installation step, updted readme as debug notes
  * adding ignore keys to readme
* Merge pull request `#26 <https://github.com/osrf/rmf_demos/issues/26>`_ from osrf/feature/caddy
  Feature/caddy
* oops, missed a conflict resolution
* merge conflict
* Fixes
* Merge pull request `#25 <https://github.com/osrf/rmf_demos/issues/25>`_ from osrf/feature/spawn-robots
  Feature/spawn robots
* change to using shell script to prevent race condition, TODO check delays on laptops
* Cleanup
* starting work on delayed loop requester
* Modified caddy drive parameters and introduced rmf_traffic dependency
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
* spawning magnis outside of charging room to save some time on getting out
* fixed duplicate waypoint name
* organized launch files, fixed nav graph number, lane mistakes
* fixed office launch missing arg, added more loops in launch, cleaned up waypoints, added new fleet lanes
* added basic airport terminal demo
* Merge pull request `#15 <https://github.com/osrf/rmf_demos/issues/15>`_ from osrf/fix_models
  Add models
* Added models
* Merge branch 'master' into feature/fix-readme
* Fixed merge conflict
* Added coke model
* Merge pull request `#12 <https://github.com/osrf/rmf_demos/issues/12>`_ from osrf/update_editor
  Added more assets to office world
* Added more assets to office world
* Fixed merge conflict
* Fixed dispenser name
* Change .traffic-editor. to .building.
* Merge pull request `#9 <https://github.com/osrf/rmf_demos/issues/9>`_ from osrf/upgrade_editor_file_format
  tweak building filename suffix and add editor project file
* Added beverage collector dispenser
* tweak building filename suffix and add editor project file
* Added office_loop launch
* Install gazebo during build action
* Added door supervisor
* Updated collision geometry for AdjTable and minor layout tweaks
* Added more lanes and magni spawn locations
* Added more doors
* Fixed linker error with door plugin
* Fixed CMake
* Merge pull request `#8 <https://github.com/osrf/rmf_demos/issues/8>`_ from osrf/feature/add_world
  Feature/add world
* Added lanes
* Formatting Fixes
* Updated office
* Updated office map
* Added model files
* Fixed media and updated office editor
* Added launch file to bringup office world
* first commit
* Contributors: Aaron, Aaron Chong, Boon Han, Charayaphan Nakorn Boon Han, Morgan Quigley, Yadu, Yadunund, ngwk1
