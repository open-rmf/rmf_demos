# rmf_demos

This package contains documentation and demo packages on running instances of
`rmf`, using core libraries such as

* [rmf_core](https://github.com/osrf/rmf_core)

* [rmf_schedule_visualizer](https://github.com/osrf/rmf_schedule_visualizer)

* [traffic_editor](https://github.com/osrf/traffic_editor)

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

## Minimal fleet demo

```bash
source ~/rmf_demos_ws/install/setup.bash
ros2 launch rmf_demo_fleets fake_read_only_fleet.launch.xml
```

