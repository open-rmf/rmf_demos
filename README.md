# RMF Demos

![](https://github.com/osrf/rmf_demos/workflows/build/badge.svg)

This repository contains documentation and demo packages which showcase the capabilities of the Robotics Middleware Framework (RMF) including traffic_management, conflict resolution, and door/lift interaction with heterogeneous robot fleets. The demos serve as a starting point for working and integrating with RMF.

![](media/loop_request.gif)

Note: The entire RMF ecosystem is still under active development, which may cause documentation, API or ABI compatibility to break. 

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
  libwebsocketpp-dev \
  libboost-all-dev curl \
  python3-shapely python3-yaml
```

Setup a new ROS 2 workspace and pull in all the required repositories using `vcs`,

```bash
mkdir -p ~/rmf_demos_ws/src
cd ~/rmf_demos_ws
wget https://raw.githubusercontent.com/osrf/rmf_demos/master/rmf_demos.repos
vcs import src < rmf_demos.repos
```

Ensure all ROS prerequisites are fulfilled,

```bash
cd ~/rmf_demos_ws
rosdep install --from-paths src --ignore-src --rosdistro eloquent \
    -y --skip-keys "websocketpp ament_python"
```

## Compiling Instructions

Source ROS 2 Eloquent and build,

```bash
cd ~/rmf_demos_ws
source /opt/ros/eloquent/setup.bash
colcon build
```

## Office World
An indoor office environemnt for robots to navigate around. It includes a beverage dispensing station, controllable doors and laneways which are integrated into RMF.


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

## Example Map

An example map/graph configuration generated using the layout of the OSRF Singapore office, has been included in the package `rmf_demo_maps`, and it can be found [here](rmf_demo_maps/maps/office/). This configuration file can be read and modified by the [traffic editor](https://github.com/osrf/traffic_editor), and is parsed by the different core packages of RMF during build time.

Below is a screenshot of how the provided demo map will look like, when opened using the `traffic editor`,

<img src="media/office_screenshot.png" width="800px"/>

## Airport Terminal World

This demo world shows robot interaction on a much larger map, with a lot more lanes, destinations, robots and possible interactions between robots from different fleets, robots and infrastructure, as well as robots and users. In the illustrations below, from top to bottom we have how the world looks like in `traffic_editor`, the schedule visualizer in `rviz`, and the full simulation in `gazebo`,

![](media/airport_terminal_traffic_editor_screenshot.png)
![](media/airport_terminal_demo_screenshot.png)

To launch the world and the schedule visualizer,

```bash
source ~/rmf_demos_ws/install/setup.bash
ros2 launch demos airport_terminal.launch.xml
```

To start a basic setup where Magni and MiR100 robots are spawned, without sending any requests,

```bash
source ~/rmf_demos_ws/install/setup.bash
ros2 run demos airport_terminal_spawn_robots.sh
```

To spawn the robots as well as sending looping jobs,

```bash
source ~/rmf_demos_ws/install/setup.bash
ros2 run demos airport_terminal_loop_scenario.sh
```

# Notes and known issues

* More instructions on using the `traffic_editor` can be found in the [repository](https://github.com/osrf/traffic_editor).

* If you encounter problems launching the demos in `gazebo`, consider removing the local installation and reinstalling using the `rosdep` command listed at the top,

```bash
sudo apt remove gazebo*

cd ~/rmf_demos_ws
rosdep install --from-paths src --ignore-src --rosdistro eloquent \
    -y --skip-keys "websocketpp ament_python"
```
