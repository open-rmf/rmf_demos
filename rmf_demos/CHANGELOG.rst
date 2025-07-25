^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rmf_demos
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.8.0 (2025-07-21)
------------------
* Add mutex group supervisor to common launch file (`#310 <https://github.com/open-rmf/rmf_demos/issues/310>`_)
  Co-authored-by: yadunund <yadunund@intrinsic.ai>
* Contributors: Grey

2.7.0 (2025-06-09)
------------------

2.6.0 (2025-05-09)
------------------
* Dynamic events CLI scripts (`#296 <https://github.com/open-rmf/rmf_demos/issues/296>`_)
  Co-authored-by: tanjunkiat <tanjunkiat@outlook.sg>
* Add missing dep for rmf_reservation_node (`#297 <https://github.com/open-rmf/rmf_demos/issues/297>`_)
* Contributors: Aaron Chong, Grey

2.5.0 (2024-11-27)
------------------
* Provide example for robot-specific finishing request (`#255 <https://github.com/open-rmf/rmf_demos/issues/255>`_)
* Add proto-reservation node as core part of RMF (`#212 <https://github.com/open-rmf/rmf_demos/issues/212>`_)
* Contributors: Arjo Chakravarty, Xiyu

2.4.0 (2024-06-12)
------------------

2.3.0 (2024-06-12)
------------------
* Remove rmf_demos_panel (`#235 <https://github.com/open-rmf/rmf_demos/pull/235>`_)
* Use same empty string default value as fleet adapter launches (`#237 <https://github.com/open-rmf/rmf_demos/pull/237>`_)
* Adding server_uri for dispatcher node (`#230 <https://github.com/open-rmf/rmf_demos/pull/230>`_)
* Add task reassignment trigger in update loop (`#228 <https://github.com/open-rmf/rmf_demos/pull/228>`_)
* Use unique hex strings for new task IDs (`#205 <https://github.com/open-rmf/rmf_demos/pull/205>`_)
* Fix airport terminal demo (`#204 <https://github.com/open-rmf/rmf_demos/pull/204>`_)
* Contributors: Aaron Chong, Luca Della Vedova, Xiyu, Yadunund

2.2.3 (2023-12-20)
------------------

2.2.2 (2023-08-28)
------------------
* EasyFullControl integration with rmf_demos (`#158 <https://github.com/open-rmf/rmf_demos/pull/158>`_)
* Contributors: Aaron Chong, Grey, Xiyu, Yadunund

2.2.1 (2023-08-10)
------------------

2.2.0 (2023-06-08)
------------------

2.1.0 (2023-06-06)
------------------
* Switch to rst changelogs (`#182 <https://github.com/open-rmf/rmf_demos/pull/182>`_)
* Version updates from latest release synced to main (`#167 <https://github.com/open-rmf/rmf_demos/pull/167>`_)
* Contributors: Esteban Martinena Guerrero, Yadunund

2.0.2 (2022-10-10)
------------------

2.0.1 (2022-09-29)
------------------

1.4.0 (2022-02-14)
------------------
* Add an indoor-outdoor campus example (`#121 <https://github.com/open-rmf/rmf_demos/pull/121>`_)

1.3.1 (2021-09-22)
------------------
* migrate rmf_panel js component to a new repo (`#93 <https://github.com/open-rmf/rmf_demos/pull/93>`_)

1.3.0 (2021-09-08)
------------------
* Add finishing_request parameter to fleet adapters: (`#83 <https://github.com/open-rmf/rmf_demos/pull/83>`_)
* Split launch files for ignition and gazebo simulators: (`#78 <https://github.com/open-rmf/rmf_demos/pull/77>`_), (`#80 <https://github.com/open-rmf/rmf_demos/pull/80>`_)
* Add reversible parameter to deliveryRobot and tinyRobot adapters: (`#74 <https://github.com/open-rmf/rmf_demos/pull/74>`_)
* Fix crash in airport terminal world due to required attribute: (`#69 <https://github.com/open-rmf/rmf_demos/pull/69>`_)

1.2.0 (2021-07-21)
------------------
* Secured Ros2 office demo. (`#135 <https://github.com/osrf/rmf_demos/pull/135>`_)
* Traffic Light Scenarios. (`#168 <https://github.com/osrf/rmf_demos/pull/168>`_)
* Rename demos package to rmf_demos
* update all namings with rmf_demos as prefix (`#1 <https://github.com/open-rmf/rmf_demos/pull/1>`_)
* Match renamed packages (`#4 <https://github.com/open-rmf/rmf_demos/pull/4>`_)
* Cleaning task demo in Airport terminal (`#8 <https://github.com/open-rmf/rmf_demos/pull/8>`_)
* Separate nav graphs for CleanerBotA and CleanerBotE (`#10 <https://github.com/open-rmf/rmf_demos/pull/10>`_)
* Reduce tinyRobot vicinity to be less disruptive (`#13 <https://github.com/open-rmf/rmf_demos/pull/13>`_)
* rename dependency following recent refactor (`#17 <https://github.com/open-rmf/rmf_demos/pull/17>`_)
* Update ignition launch for new sim plugin names (`#19 <https://github.com/open-rmf/rmf_demos/pull/19>`_)
* Feature/add crowdsim example (`#14 <https://github.com/open-rmf/rmf_demos/pull/14>`_)
* Remove server.launch.xml (`#11 <https://github.com/open-rmf/rmf_demos/pull/11>`_)
* fix caddy discover timeout (`#23 <https://github.com/open-rmf/rmf_demos/pull/23>`_)
* Changes after renaming packages in rmf_visualization (`#26 <https://github.com/open-rmf/rmf_demos/pull/26>`_)
* refactoring to bring use_sim_time to a single global definition (`#31 <https://github.com/open-rmf/rmf_demos/pull/31>`_)

1.1.0 (2020-09-23)
------------------
* Update dependency. (`#101 <https://github.com/osrf/rmf_demos/pull/101>`_)
* Support demos on ROS 2 Foxy and Eloquent. (`#103 <https://github.com/osrf/rmf_demos/pull/103>`_)
* Add subscription to floorplan topic published by rmf_schedule_visualizer. (`#133 <https://github.com/osrf/rmf_demos/pull/133>`_)
* Added `Triple H` scenario and `The Pedigree` conflict launch file: (`#139 <https://github.com/osrf/rmf_demos/pull/139>`_)
* Contributors: Aaron Chong, Grey, Kevin_Skywalker, Michael X. Grey, Yadu, ddengster, kevinskwk, mrushyendra, youliang

1.0.0 (2020-06-24)
------------------
* Provides launch files to launch the `Office` and `Airport Terminal` simulations worlds along with the necessary RMF backend services.
* Contributors: Aaron, Aaron Chong, Boon Han, Charayaphan Nakorn Boon Han, Morgan Quigley, Yadu, Yadunund
