# RMF Demos

![](https://github.com/osrf/rmf_demos/workflows/build/badge.svg)

This repository contains documentation and demo packages which showcase the capabilities of the Robotics Middleware Framework (RMF) including traffic_management, conflict resolution, and door/lift interaction with heterogeneous robot fleets. The demos serve as a starting point for working and integrating with RMF.

[![Robotics Middleware Framework](media/thumbnail.png)](https://vimeo.com/405803151)

(Click to watch video)

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

Some of the demos might refer to open source gazebo models hosted [here](https://github.com/osrf/gazebo_models). To avoid any race conditions and errors where `gazebo` downloads all models when launched, optionally, a local copy of all the open source models can be downloaded and saved manually like so,

```bash
cd ~/.
git clone https://github.com/osrf/gazebo_models
cd gazebo_models
cp -r ./* ~/.gazebo/models/.
```

## Compiling Instructions

Source ROS 2 Eloquent and build,

```bash
cd ~/rmf_demos_ws
source /opt/ros/eloquent/setup.bash
colcon build
```

# Office World
An indoor office environemnt for robots to navigate around. It includes a beverage dispensing station, controllable doors and laneways which are integrated into RMF.


```bash
source ~/rmf_demos_ws/install/setup.bash
ros2 launch demos office.launch.xml
```

To simulate a delivery

Click the `Request Delivery` button in the RViz `RMF Panel` or

```bash
source ~/rmf_demos_ws/install/setup.bash
ros2 run rmf_demo_tasks request_delivery 
``` 

![](media/delivery_request.gif)

To request each of the Magni robots to loop between two points,

Click the `Request Loop` button in the RViz `RMF Panel` or

```bash
source ~/rmf_demos_ws/install/setup.bash
ros2 launch demos office_loop.launch.xml
``` 

![](media/loop_request.gif)

# Airport Terminal World

This demo world shows robot interaction on a much larger map, with a lot more lanes, destinations, robots and possible interactions between robots from different fleets, robots and infrastructure, as well as robots and users. In the illustrations below, from top to bottom we have how the world looks like in `traffic_editor`, the schedule visualizer in `rviz`, and the full simulation in `gazebo`,

![](media/airport_terminal_traffic_editor_screenshot.png)
![](media/airport_terminal_demo_screenshot.png)

## Demo Scenario
To launch the world and the schedule visualizer,

```bash
source ~/rmf_demos_ws/install/setup.bash
ros2 launch demos airport_terminal.launch.xml
```

To run a scenario where multiple robots are issued task orders,

```bash
source ~/rmf_demos_ws/install/setup.bash
ros2 run demos airport_terminal_scenario.sh
```
This command spawns 2 mir100 and 4 magni robots in the map. Out of these 1 mir100 and 2 magni robots are issued loop request tasks. The other robots are idle and can be issued loop or delivery reqeust tasks via the `RMF Panel`.

Non-autonomous vehicles can also be integrated with RMF provided their positions can be localized in the world. This may be of value at facilities where space is shared by autonomous robots as well as manually operated vechiles such as forklifts or transporters. In this demo, we can introduce a vehicle (caddy) which can be driven around through keyboard/joystick teleop. In RMF nomenclature, this vehicle is classified as a `read_only` type, ie, RMF can only infer its position in the world but does not have control over its motion. Here, the goal is to have other controllable robots avoid this vechile's path by replanning their routes if needed. The model is fitted with a plugin which generates a prediction of the vehicle's path based on its current heading. It is configured to occupy the same lanes as the `magni` robots. Here, a `read_only_fleet_adapter` submits the prediction from the plugin to the RMF schedule.

To spawn the caddy into the world,

```bash
source ~/rmf_demos_ws/install/setup.bash
ros2 launch demos airport_terminal_caddy.launch.xml
```

![](media/caddy.gif)

Alternatively, to spawn all the robots without issuing any task orders,

```bash
source ~/rmf_demos_ws/install/setup.bash
ros2 run demos airport_terminal_spawn_robots.sh
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
