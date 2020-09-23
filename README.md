# RMF Demos

![](https://github.com/osrf/rmf_demos/workflows/build/badge.svg)
![](https://github.com/osrf/rmf_demos/workflows/style/badge.svg)

The Robotics Middleware Framework (RMF) enables interoperability among heterogeneous robot fleets while managing robot traffic that share resources such as space, building infrastructure systems (lifts, doors, etc) and other automation systems within the same facility. RMF also handles task allocation and conflict resolution  among its participants (de-conflicting traffic lanes and other resources). These capabilities are provided by various libraries in [rmf_core](https://github.com/osrf/rmf_core).

This repository contains demonstrations of the above mentioned capabilities of RMF. It serves as a starting point for working and integrating with RMF.

[![Robotics Middleware Framework](docs/media/thumbnail.png)](https://vimeo.com/405803151)

#### (Click to watch video)

## System Requirements

These demos were developed and tested on

* [Ubuntu 20.04 LTS](https://releases.ubuntu.com/20.04/)

* [ROS 2 - Foxy](https://index.ros.org/doc/ros2/Releases/Release-Foxy-Fitzroy/)

* [Gazebo 11.1.0](http://gazebosim.org/)

## Installation
Instructions can be found [here](docs/installation.md).

## FAQ
Answers to frequently asked questions can be found [here](docs/faq.md).

## Demo Worlds

* [Office World](#Office-World)
* [Airport Terminal World](#Airport-Terminal-World)
* [Clinic World](#Clinic-World)
* [Hotel World](#Hotel-World)

> Note: When running the demos on Ubuntu 18.04 (not officially supported), you are required to explicitly supply gazebo_version launch argument. Eg:
ros2 launch demos office.launch.xml gazebo_version:=9

---

### Office World
An indoor office environment for robots to navigate around. It includes a beverage dispensing station, controllable doors and laneways which are integrated into RMF.

```bash
source ~/rmf_demos_ws/install/setup.bash
ros2 launch demos office.launch.xml
```

To simulate a delivery

Select `pantry` and `hardware_2` as `Start` and `End` waypoints. Ensure `Pickup` and `Dropoff` dispensers are set to `coke_dispenser` and `coke_ingestor` respectively. Finally, click the `Send Delivery Request` button in the RViz `RMF Panel`.

Alternatively, a launch file is configured to achieve the same result.

```bash
source ~/rmf_demos_ws/install/setup.bash
ros2 launch demos office_delivery.launch.xml 
``` 

![](docs/media/delivery_request.gif)

To request each of the TinyRobot to loop between two points,
Select desired `Start` and `End` waypoints using the `RMF Panel` and click the `Send Loop Request` button. Alternatively,

```bash
source ~/rmf_demos_ws/install/setup.bash
ros2 launch demos office_loop.launch.xml
``` 

![](docs/media/loop_request.gif)

---

### Airport Terminal World

This demo world shows robot interaction on a much larger map, with a lot more lanes, destinations, robots and possible interactions between robots from different fleets, robots and infrastructure, as well as robots and users. In the illustrations below, from top to bottom we have how the world looks like in `traffic_editor`, the schedule visualizer in `rviz`, and the full simulation in `gazebo`,

![](docs/media/airport_terminal_traffic_editor_screenshot.png)
![](docs/media/airport_terminal_demo_screenshot.png)

#### Demo Scenario
To launch the world and the schedule visualizer,

```bash
source ~/rmf_demos_ws/install/setup.bash
ros2 launch demos airport_terminal.launch.xml
```

To spawn robots into the world and issue tasks to the same,

```bash
source ~/rmf_demos_ws/install/setup.bash
ros2 run demos airport_terminal_scenario.sh
```
This command spawns two DeliveryRobots and four TinyRobots in the map. Out of these one DeliveryRobot and two TinyRobots are issued loop request tasks. The other robots are idle and can be issued loop or delivery request tasks via the `RMF Panel`.

Alternatively, to spawn all the robots without issuing any task orders,

```bash
source ~/rmf_demos_ws/install/setup.bash
ros2 run demos airport_terminal_spawn_robots.sh
```

An delivery request may also be submitted via a launch file,
```bash
source ~/rmf_demos_ws/install/setup.bash
ros2 launch demos airport_terminal_delivery.launch.xml
```

Non-autonomous vehicles can also be integrated with RMF provided their positions can be localized in the world. This may be of value at facilities where space is shared by autonomous robots as well as manually operated vechiles such as forklifts or transporters. In this demo, we can introduce a vehicle (caddy) which can be driven around through keyboard/joystick teleop. In RMF nomenclature, this vehicle is classified as a `read_only` type, ie, RMF can only infer its position in the world but does not have control over its motion. Here, the goal is to have other controllable robots avoid this vechile's path by replanning their routes if needed. The model is fitted with a plugin which generates a prediction of the vehicle's path based on its current heading. It is configured to occupy the same lanes as the `tinyRobot` robots. Here, a `read_only_fleet_adapter` submits the prediction from the plugin to the RMF schedule.

To spawn the caddy into the world,

```bash
source ~/rmf_demos_ws/install/setup.bash
ros2 launch demos airport_terminal_caddy.launch.xml
```

![](docs/media/caddy.gif)

---

### Clinic World

This is an imaginary clinic building with three levels and two lifts. Three different robot fleets with different roles navigate across all three levels by lifts. In the illustrations below, we have the view of all three levels in `traffic_editor` (top left), the schedule visualizer in `rviz` (right), and the full simulation in `gazebo` (bottom left).

![](docs/media/clinic.png)

#### Demo Scenario
To launch the world and the schedule visualizer,

```bash
source ~/rmf_demos_ws/install/setup.bash
ros2 launch demos clinic.launch.xml
```

To simulate a delivery

Select fleet `tinyRobot`, set `L1_pantry` and `L3_ward4` as `Start` and `End` waypoints. Ensure `Pickup` and `Dropoff` dispensers are set to `coke_dispenser` and `coke_ingestor` respectively. Finally, click the `Send Delivery Request` button in the RViz `RMF Panel`.

Alternatively, a launch file is configured to achieve the same result.

```bash
source ~/rmf_demos_ws/install/setup.bash
ros2 launch demos clinic_delivery.launch.xml 
``` 

To request each of the robots to loop between two points, select desired robot fleet, `Start` and `End` waypoints using the `RMF Panel` and click the `Send Loop Request` button. Alternatively,

```bash
source ~/rmf_demos_ws/install/setup.bash
ros2 launch demos clinic_loop.launch.xml
``` 

Robots taking lift:
![](docs/media/robots_taking_lift.gif)

Multi-fleet demo:
![](docs/media/clinic.gif)

---

### Hotel World

This is a hotel with a lobby and a guest level. The hotel has two lifts and two robot fleets. The tiny robots are supposed to guide the guests and the delivery robots are used to load and deliver cargo.

The hotel map is truncated due to the high memory usage. The full map can be accessed [here](https://github.com/MakinoharaShouko/hotel).

Hotel floor plan in `traffic_editor`:
![](docs/media/hotel.png)

Full hotel floor plan in `traffic_editor`:
![](docs/media/hotel_full.png)

#### Demo Scenario

To launch the world and the schedule visualizer,

```bash
source ~/rmf_demos_ws/install/setup.bash
ros2 launch demos hotel.launch.xml
```

To simulate a loop request, select desired robot fleet, `Start` and `End` waypoints using the `RMF Panel` and click the `Send Loop Request` button.

Robot taking lift:

![](docs/media/robot_taking_lift_hotel.gif)
