^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rmf_demos_maps
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.8.0 (2025-07-21)
------------------
* Use builtin gazebo model downloading (`#307 <https://github.com/open-rmf/rmf_demos/issues/307>`_)
* Contributors: Luca Della Vedova

2.7.0 (2025-06-09)
------------------

2.6.0 (2025-05-09)
------------------
* Change robot types to include a namespace (`#272 <https://github.com/open-rmf/rmf_demos/issues/272>`_)
* Contributors: Luca Della Vedova

2.5.0 (2024-11-27)
------------------
* Add proto-reservation node as core part of RMF (`#212 <https://github.com/open-rmf/rmf_demos/issues/212>`_)
* Remove nav graph for fleet manager and update Office world (`#263 <https://github.com/open-rmf/rmf_demos/issues/263>`_)
* Contributors: Arjo Chakravarty, Xiyu

2.4.0 (2024-06-12)
------------------

2.3.0 (2024-06-12)
------------------
* Port to harmonic (`#206 <https://github.com/open-rmf/rmf_demos/pull/206>`_)
* Add a script for opening and closing lanes (`#216 <https://github.com/open-rmf/rmf_demos/pull/216>`_)
* Fix office dispenser height to be on top of furniture (`#218 <https://github.com/open-rmf/rmf_demos/pull/218>`_)
* Remove a few assets from the office world (`#209 <https://github.com/open-rmf/rmf_demos/pull/209>`_)
* Update Office visuals and add aligned laser scan layer (`#202 <https://github.com/open-rmf/rmf_demos/pull/202>`_)
* Contributors: Grey, Luca Della Vedova, Yadunund

2.2.3 (2023-12-20)
------------------

2.2.2 (2023-08-28)
------------------
* EasyFullControl integration with rmf_demos (`#158 <https://github.com/open-rmf/rmf_demos/pull/158>`_)
* Contributors: Aaron Chong, Grey, Xiyu, Yadunund

2.2.1 (2023-08-10)
------------------
* Fix gz classic model download (`#185 <https://github.com/open-rmf/rmf_demos/pull/185>`_)
* Contributors: Aaron Chong

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
* Async behavior improvements (`#157 <https://github.com/open-rmf/rmf_demos/pull/157>`_)
* Feature/demo fleet adapter (`#109 <https://github.com/open-rmf/rmf_demos/pull/109>`_)
* Additional composed task scripts (`#129 <https://github.com/open-rmf/rmf_demos/pull/129>`_)
* Helpful scripts for submitting composite tasks (`#122 <https://github.com/open-rmf/rmf_demos/pull/122>`_)
* Add an indoor-outdoor campus example (`#121 <https://github.com/open-rmf/rmf_demos/pull/121>`_)

1.2.0 (2021-07-21)
------------------
* update all namings with rmf_demos as prefix (`#1 <https://github.com/open-rmf/rmf_demos/pull/1>`_)
* Cleaning task demo in Airport terminal (`#8 <https://github.com/open-rmf/rmf_demos/pull/8>`_)
* Separate nav graphs for CleanerBotA and CleanerBotE (`#10 <https://github.com/open-rmf/rmf_demos/pull/10>`_)
* rename dependency following recent refactor (`#17 <https://github.com/open-rmf/rmf_demos/pull/17>`_)
* Feature/add crowdsim example (`#14 <https://github.com/open-rmf/rmf_demos/pull/14>`_)

1.1.0 (2020-09-24)
------------------
* Triple H Scenario
* Clinic dashboard fixes (`#136 <https://github.com/osrf/rmf_demos/pull/136>`_)
* Adjusted model orientations. (`#90 <https://github.com/osrf/rmf_demos/pull/90>`_)
* Fix build failure in buildfarm. (`#96 <https://github.com/osrf/rmf_demos/pull/96>`_)
* CMake variable NO_DONWLOAD_MODELS used to download models during build time. (`#98 <https://github.com/osrf/rmf_demos/pull/98>`_)
* Switched to using DeliveryRobot and TinyRobot models. (`#115 <https://github.com/osrf/rmf_demos/pull/115>`_)
* Add Clinic and Hotel multi-level maps for showcasing lifts in demonstrations. (`#120 <https://github.com/osrf/rmf_demos/pull/120>`_)
* Contributors: Grey, Kevin_Skywalker, Michael X. Grey, Yadu, ddengster, kevinskwk

1.0.0 (2020-06-24)
------------------
* Provides an annotated `traffic_editor` map for the `Office` world simulation which contains two TinyRobots operating on a single traffic graph, a beverage dispensing station and two motorized doors
* Provides an annotated `traffic_editor` map for the `Airport Terminal` world simulation which contains four TinyRobots and two DeliveryRobots operating on separate traffic graphs. The map also includes several motorized doors as well as a cleaning supplies dispensing station
* Contributors: Aaron, Aaron Chong, Boon Han, Charayaphan Nakorn Boon Han, Morgan Quigley, Yadu, Yadunund, ngwk1
