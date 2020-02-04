# RMF Demos

![](https://github.com/osrf/rmf_demos/workflows/build/badge.svg)

This repository contains documentation and demo packages on running instances of RMF. The demos will serve as a starting point, for working and integrating with RMF.

Do take note that the entire RMF ecosystem is still under active development, which might sometimes cause documentation, API or ABI compatibility to break. 

## System Requirements

These demos were developed and tested on

* [Ubuntu 18.04 LTS](https://releases.ubuntu.com/18.04/)

* [ROS 2 - Eloquent](https://index.ros.org/doc/ros2/Releases/Release-Eloquent-Elusor/)

## Setup

Install all non-ROS prerequisites of the packages,

```bash
sudo apt update && sudo apt install  \
  git cmake wget python3-vcstool \
  qt5-default libeigen3-dev \
  libccd-dev libfcl-dev \
  libyaml-cpp-dev libwebsocketpp-dev \
  libboost-all-dev
```

Ensure all ROS prerequisites are fulfilled,

```bash
sudo apt install \
  ros-eloquent-rviz2 \
  ros-eloquent-launch*
```

## Compiling Instructions

Setup a new ROS 2 workspace and pull in all the required repositories using `vsc`,

```bash
mkdir -p ~/rmf_demos_ws/src
cd ~/rmf_demos_ws
wget https://raw.githubusercontent.com/osrf/rmf_demos/master/rmf_demos.repos
vcs import src < rmf_demos.repos
```

Source ROS 2 Eloquent and build,

```bash
cd ~/rmf_demos_ws
source /opt/ros/eloquent/setup.bash
colcon build
```

## Example Map

An example map/graph configuration generated using the layout of the OSRF Singapore office, has been included in the package `rmf_demo_maps`, and it can be found [here](rmf_demo_maps/maps/office/). This configuration file can be read and modified by the [traffic editor](https://github.com/osrf/traffic_editor), and is parsed by the different core packages of RMF during launch time.

Below is a screenshot of how the provided demo map will look like, when opened using the `traffic editor`,

<img src="media/office_screenshot.png" width="800px"/>

More instructions on using the `traffic_editor` can be found in the [repository](https://github.com/osrf/traffic_editor).

## Minimal fleet demo

This minimal demonstration launches a string of core `rmf` programs, which includes the internal map server, the traffic scheduler,  a fake fleet with one robot, its corresponding fleet adapter, as well as the overall schedule visualizer. A read-only fleet will not allow `rmf_core` or any other RMF-related systems to control it in any way. It only reports its status upstream to RMF, where traffic scheduling will ensure other fleets' traffic is diverted away.

Launch the demo using the launch file below,

```bash
source ~/rmf_demos_ws/install/setup.bash
ros2 launch rmf_demo_fleets fake_read_only_fleet.launch.xml
```

When the launch is done, a seemingly empty`rviz2` will have been opened. Toggle the left panel arrow to open up the scheduler panel and change the field `map_name` to `L1`. Zoom out and the visualizer should show the lanes and robot waypoints corresponding to the example map of OSRF Singapore office.

<img src="media/office_rviz.png" width="800px"/>

There will be a 180 degree angle offset, which can be fixed by further parameterizing the demo map.

The fake robot in the launched fake fleet, can also be seen moving diagonally away from the center of the map, indicated by a small purple marker.

In order to shut it down, use `Ctrl+C` on the terminal that launched the system via `ros2 launch ...`.
