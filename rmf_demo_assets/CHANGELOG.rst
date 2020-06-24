^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rmf_demo_assets
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------

1.0.0 (2020-06-24)
------------------
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
* Disabled caddy merge lane
* Modified motion params of Mir100
* Undo change to RobotPlaceholder
* Added more doors
* Merge pull request `#60 <https://github.com/osrf/rmf_demos/issues/60>`_ from osrf/fix/wood-material
  fixed duplicate wood texture
* fixed duplicate wood texture
* Removed Bookshelf asset
* Added missing bookshelf asset.
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
* Minor enhancement to readonly plugin (`#52 <https://github.com/osrf/rmf_demos/issues/52>`_)
  * POC merge lane
  * POC merge lane
  * Added lane_threshold to control merging of robot into its lane
  * Spawning caddy without launching airport_terminal world
  * Format fix
* added extra demo assets, TODO replace with model download pipeline (`#46 <https://github.com/osrf/rmf_demos/issues/46>`_)
* added some basic instructions to solve missing models race-condition … (`#39 <https://github.com/osrf/rmf_demos/issues/39>`_)
  * added some basic instructions to solve missing models race-condition when launching demos
  * typo :skull:
  * added link
  * modifying Caddy model paths instead
* Merge pull request `#26 <https://github.com/osrf/rmf_demos/issues/26>`_ from osrf/feature/caddy
  Feature/caddy
* merge conflict
* Merge pull request `#25 <https://github.com/osrf/rmf_demos/issues/25>`_ from osrf/feature/spawn-robots
  Feature/spawn robots
* Plugin now predicts look_ahead waypoints
* Modified caddy drive parameters and introduced rmf_traffic dependency
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
* spawning magnis outside of charging room to save some time on getting out
* added basic airport terminal demo
* Merge pull request `#15 <https://github.com/osrf/rmf_demos/issues/15>`_ from osrf/fix_models
  Add models
* Added models
* Merge branch 'master' into feature/fix-readme
* Fixed merge conflict
* Load and unload of can
* Added teleport plugin to coke model
* Added coke model
* Fixed merge conflict
* Merge pull request `#10 <https://github.com/osrf/rmf_demos/issues/10>`_ from osrf/add_models
  some new office models
* some new office models
* Updated collision geometry for AdjTable and minor layout tweaks
* Merge pull request `#8 <https://github.com/osrf/rmf_demos/issues/8>`_ from osrf/feature/add_world
  Feature/add world
* Added rmf_gazebo_plugins
* Formatting Fixes
* Added Magni model
* Updated office map
* Fixed texture
* add OfficeChairGrey
* Added model files
* Fixed media and updated office editor
* Added assets for demos
* Contributors: Aaron, Aaron Chong, Charayaphan Nakorn Boon Han, Morgan Quigley, Yadu, Yadunund, ngwk1
