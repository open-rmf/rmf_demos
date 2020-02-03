# RMF Demos

This repository contains documentation and demo packages on running instances of `rmf`. The demos will serve as a starting point, for working and integrating with `rmf`.

Do take note that the entire `rmf` ecosystem is still under active development, which might sometimes cause documentation, API or ABI compatibility to break. 

## System Requirements

These demos were developed and tested on

* [Ubuntu 18.04 LTS](https://releases.ubuntu.com/18.04/)

* [ROS 2 - Eloquent](https://index.ros.org/doc/ros2/Releases/Release-Eloquent-Elusor/)

## Setup

Install all prerequisites of the packages,

```bash
sudo apt update

sudo apt install git cmake libyaml-cpp-dev qt5-default libeigen3-dev libccd-dev libfcl-dev libyaml-cpp-dev ros-eloquent-rviz2 libwebsocketpp-dev libboost-all-dev -y
```

## Compiling Instructions

Setup a new ROS 2 workspace,

```bash
mkdir -p ~/rmf_demos_ws/src
cd ~/rmf_demos_ws/src

git clone https://github.com/osrf/rmf_core.git
git clone https://github.com/osrf/traffic_editor.git
git clone https://github.com/osrf/rmf_schedule_visualizer.git
git clone https://github.com/osrf/rmf_demos.git
git clone https://github.com/ros2/launch.git
git clone https://github.com/ros2/launch_ros.git
```

Source ROS 2 Eloquent and build,

```bash
cd ~/rmf_demos_ws
source /opt/ros/eloquent/setup.bash
colcon build
```

## Example Map

An example map/graph configuration generated using the layout of the OSRF Singapore office, has been included in the package `rmf_demo_maps`, it can be found [here](rmf_demo_maps/maps/office/). This configuration file can be read and modified by the [traffic editor](https://github.com/osrf/traffic_editor), and is parsed by the different core packages of `rmf` during launch time.

## Minimal fleet demo

This minimal demonstration launches a string of core `rmf` programs, which includes the internal map server, the traffic scheduler,  a fake fleet with one robot, its corresponding fleet adapter, as well as the overall schedule visualizer.

Launch the demo using the launch file below,

```bash
source ~/rmf_demos_ws/install/setup.bash
ros2 launch rmf_demo_fleets fake_read_only_fleet.launch.xml
```

When the launch is done, a seemingly empty`rviz2` will have been opened. Toggle the left panel arrow to open up the scheduler panel and change the field `map_name` to `L1`. Zoom out and the visualizer should show the lanes and robot waypoints corresponding to the example map of OSRF Singapore office.

The fake robot in the launched fake fleet, can also be seen moving diagonally away from the center of the map, indicated by a small marker.

