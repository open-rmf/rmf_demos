# RMF Demos

![](https://github.com/osrf/rmf_demos/workflows/build/badge.svg)

This repository contains documentation and demo packages on running instances of RMF. The demos will serve as a starting point, for working and integrating with RMF.

Do take note that the entire RMF ecosystem is still under active development, which might sometimes cause documentation, API or ABI compatibility to break. 

## System Requirements

These demos were developed and tested on

* [Ubuntu 18.04 LTS](https://releases.ubuntu.com/18.04/)

* [ROS 2 - Eloquent](https://index.ros.org/doc/ros2/Releases/Release-Eloquent-Elusor/)

* [Gazebo](http://gazebosim.org/blog/gazebo9)

## Setup

Install all non-ROS prerequisites of the packages,

```bash
sudo apt update && sudo apt install \
  git cmake wget python3-vcstool \
  qt5-default libeigen3-dev \
  libccd-dev libfcl-dev \
  libyaml-cpp-dev libwebsocketpp-dev \
  libboost-all-dev curl \
  python3-shapely python3-yaml
```

Install Gazebo
```bash
# this step is optional, but if you do encounter issues with your already
# existing gazebo build, you should consider removing it before reinstalling
sudo apt remove gazebo*

curl -sSL http://get.gazebosim.org | sh
```

Ensure all ROS prerequisites are fulfilled,

```bash
sudo apt install \
  ros-eloquent-rviz2 \
  ros-eloquent-launch* \
  ros-eloquent-gazebo-ros-pkgs
```

## Compiling Instructions

Setup a new ROS 2 workspace and pull in all the required repositories using `vcs`,

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

## Office World

This repository contains a demo world along with tools to examine capabilites of `rmf_core` including traffic_management, conflict resolution, door integration among others. 

```bash
source ~/rmf_demos_ws/install/setup.bash
ros2 launch demos office.launch.xml
```

To simulate a delivery
```bash
source ~/rmf_demos_ws/install/setup.bash
ros2 run rmf_demo_tasks request_delivery 
``` 
![](media/delivery_request.gif)

To request each of the Magni robots to loop between two points,

```bash
source ~/rmf_demos_ws/install/setup.bash
ros2 launch demos office_loop.launch.xml
``` 
![](media/loop_request.gif)

## Example Map

An example map/graph configuration generated using the layout of the OSRF Singapore office, has been included in the package `rmf_demo_maps`, and it can be found [here](rmf_demo_maps/maps/office/). This configuration file can be read and modified by the [traffic editor](https://github.com/osrf/traffic_editor), and is parsed by the different core packages of RMF during build time.

Below is a screenshot of how the provided demo map will look like, when opened using the `traffic editor`,

<img src="media/office_screenshot.png" width="800px"/>

## Airport Terminal World

This demo world shows robot interaction on a much larger map, with a lot more lanes, destinations, robots and possible interactions between robots from different fleets, robots and infrastructure, as well as robots and users.

```bash
source ~/rmf_demos_ws/install/setup.bash
ros2 launch demos airport_terminal.launch.xml
```

![](media/airport_terminal_screenshot.png)

To start a basic setup where Magni and MiR100 robots are spawned and sent looping jobs,

```bash
source ~/rmf_demos_ws/install/setup.bash
cd ~/rmf_demos_ws/demos/scripts/airport_terminal
./airport_terminal_loop_scenario.sh
```

More instructions on using the `traffic_editor` can be found in the [repository](https://github.com/osrf/traffic_editor).
