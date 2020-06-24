^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rmf_demo_tasks
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------

1.0.0 (2020-06-24)
------------------
* Merge pull request `#86 <https://github.com/osrf/rmf_demos/issues/86>`_ from osrf/fix_symlink_install
  Install scripts properly in symlink installs
* Install scripts properly in symlink installs
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
* Merge remote-tracking branch 'origin/master' into new_interface
* Merge pull request `#68 <https://github.com/osrf/rmf_demos/issues/68>`_ from osrf/fix/airport_scenarios
  Fix/airport scenarios
* Feature/teleport plugin v2 (`#62 <https://github.com/osrf/rmf_demos/issues/62>`_)
  * experimenting with placeholder links on models, refactoring teleport to have dispenser stuff
  * no need for mutexes
  * fixed RobotPlaceholder model, teleport plugin works, now v3 to remove target name
  * extended respawn to 5 seconds, caching modelptrs instead of names
  * handle duplicate dispenser request guids
  * cleaned up dispenser script and usage
  * removed launch call for dispensers
* merge conflict
* Merge pull request `#25 <https://github.com/osrf/rmf_demos/issues/25>`_ from osrf/feature/spawn-robots
  Feature/spawn robots
* minor cleanup
* change to using shell script to prevent race condition, TODO check delays on laptops
* scenario launch file and delayed request
* starting work on delayed loop requester
* added spawn and loop launch scripts, WIP resolving issue of hesitant robots
* Fixed merge conflict
* Enabled magni fleet adapter to perform deliveries
* Added dispenser node
* Added delivery request task
* Added office_loop launch
* Added loop_request task
* Contributors: Aaron, Aaron Chong, Charayaphan Nakorn Boon Han, Geoffrey Biggs, Yadu, Yadunund
