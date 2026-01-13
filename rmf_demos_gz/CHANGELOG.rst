^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rmf_demos_gz
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.8.1 (2026-01-13)
------------------
* Switch to using ros_gz_sim to run Gazebo (`#344 <https://github.com/open-rmf/rmf_demos/issues/344>`_)
* Contributors: Luca Della Vedova

2.8.0 (2025-07-21)
------------------
* Use builtin gazebo model downloading (`#307 <https://github.com/open-rmf/rmf_demos/issues/307>`_)
* Contributors: Luca Della Vedova

2.7.0 (2025-06-09)
------------------
* Remove gazebo_version argument (`#305 <https://github.com/open-rmf/rmf_demos/issues/305>`_)
  * Version independent default gazebo
  * Remove gazebo version altogether
  * Remove gazebo_version from README
  ---------
* Contributors: Luca Della Vedova

2.6.0 (2025-05-09)
------------------
* Use 9 as default gazebo_version (`#292 <https://github.com/open-rmf/rmf_demos/issues/292>`_)
* Remove redundant launch file (`#271 <https://github.com/open-rmf/rmf_demos/issues/271>`_)
* Contributors: Arjo Chakravarty, Grey

2.5.0 (2024-11-27)
------------------
* Add proto-reservation node as core part of RMF (`#212 <https://github.com/open-rmf/rmf_demos/issues/212>`_)
* Adds the ability to set the update rate of the simulation (`#264 <https://github.com/open-rmf/rmf_demos/issues/264>`_)
* Use environment hooks for plugins / assets (`#262 <https://github.com/open-rmf/rmf_demos/issues/262>`_)
* Contributors: Arjo Chakravarty, Luca Della Vedova

2.4.0 (2024-06-12)
------------------

2.3.0 (2024-06-12)
------------------
* Port to harmonic (`#206 <https://github.com/open-rmf/rmf_demos/issues/206>`_)
* Moved rmf_demos to use ros_gz_bridge instead of ros_ign_bridge (`#173 <https://github.com/open-rmf/rmf_demos/issues/173>`_)
* Contributors: Arjo Chakravarty, Luca Della Vedova, Michael X. Grey, Yadunund

2.2.3 (2023-12-20)
------------------

2.2.2 (2023-08-28)
------------------

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
* Renamed to rmf_demos_gz

1.3.1 (2021-11-30)
------------------
* Added `diff_drive` plugin dependencies for caddy (`#111 <https://github.com/open-rmf/rmf_demos/pull/111>`_)
* Modified plugin path to match `rmf_simulation` install (`#108 <https://github.com/open-rmf/rmf_demos/pull/108>`_)

1.3.0 (2021-09-08)
------------------
* Provides launch files for running various demonstrations in ignition-gazebo
