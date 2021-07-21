# RMF Demos

![](https://github.com/open-rmf/rmf_demos/workflows/build/badge.svg)
![](https://github.com/open-rmf/rmf_demos/workflows/style/badge.svg)

The Robotics Middleware Framework (RMF) enables interoperability among heterogeneous robot fleets while managing robot traffic that share resources such as space, building infrastructure systems (lifts, doors, etc) and other automation systems within the same facility. RMF also handles task allocation and conflict resolution  among its participants (de-conflicting traffic lanes and other resources). These capabilities are provided by various libraries in [RMF](https://github.com/open-rmf/rmf).

This repository contains demonstrations of the above mentioned capabilities of RMF. It serves as a starting point for working and integrating with RMF.

[![Robotics Middleware Framework](../media/thumbnail.png?raw=true)](https://vimeo.com/405803151)

#### (Click to watch video)

## System Requirements

These demos were developed and tested on

* [Ubuntu 20.04 LTS](https://releases.ubuntu.com/20.04/)

* [ROS 2 - Foxy](https://index.ros.org/doc/ros2/Releases/Release-Foxy-Fitzroy/)

* [Gazebo 11.1.0](http://gazebosim.org/)

## Installation
Instructions can be found [here](https://github.com/open-rmf/rmf).

## FAQ
Answers to frequently asked questions can be found [here](docs/faq.md).

## Roadmap

A near-term roadmap of the RMF project can be found in the user manual [here](https://osrf.github.io/ros2multirobotbook/roadmap.html).

## RMF Panel
![](../media/RMF_Panel.png?raw=true)

The RMF panel is a web based dashboard for interacting with RMF. It allows users to send task requests to RMF and monitor the status of robots and submitted tasks.
There are two main modes of submitting tasks to RMF via the Panel:

1. Submit a Task: Used to submit a single task. The user is required to first select a request type from the drop down menu. Depending on the type selected, additional fields specify to the type will need to be populated. The user can then specify the `start time` for the task before clicking `Submit Request`. 
2. Submit a List of Tasks: Used to submit a batch of tasks. A `.json` file containing a list of tasks may be loaded via the `Choose file` button. Some example files are found in `rmf_demos_panel/task_lists`. Once loaded, clicking the `Submit Task List` button will automatically assign the various tasks to available robots.

Users may switch between different tabs on the top-left corner of the Panel when running the relevant demo world. More information on configuring the panel can be found [here](rmf_demos_panel/README.md)


## Demo Worlds

* [Office World](#Office-World)
* [Airport Terminal World](#Airport-Terminal-World)
* [Clinic World](#Clinic-World)
* [Hotel World](#Hotel-World)

> Note: When running the demos on Ubuntu 18.04 (not officially supported), you are required to explicitly supply gazebo_version launch argument. Eg:
ros2 launch rmf_demos office.launch.xml gazebo_version:=9

---

### Office World
An indoor office environment for robots to navigate around. It includes a beverage dispensing station, controllable doors and laneways which are integrated into RMF.

```bash
source ~/rmf_demos_ws/install/setup.bash
ros2 launch rmf_demos office.launch.xml
```

To send task requests, open RMF Panel from a browser
```bash
firefox localhost:5000
```

To submit a delivery task, select `Delivery` from the `Select a request type` dropdown list. Next, select `coke` from the `Select delivery task` list. Choose an desired start time for task and click submit.

Or, submit a task via CLI:
```bash
ros2 run rmf_demos_tasks dispatch_loop -s coe -f lounge -n 3 --use_sim_time
ros2 run rmf_demos_tasks dispatch_delivery -p pantry -pd coke_dispenser -d hardware_2 -di coke_ingestor --use_sim_time
```

![](../media/delivery_request.gif?raw=true)

To send loop requests, select `Loop` from the `Select a request type` dropdown list. Choose desired start and end locations and click submit.
To run a scenario with multiple task requests, load `office_tasks.json` from `rmf_demos_panel/task_lists` in the `Submit a list of tasks` section. This should populate the preview window with a list of tasks. Click submit and watch the demonstration unfold.

![](../media/loop_request.gif)

The office demo can be run in secure mode using ROS 2 DDS-Security integration. Click [here](docs/secure_office_world.md) to learn more.

---

### Airport Terminal World

This demo world shows robot interaction on a much larger map, with a lot more lanes, destinations, robots and possible interactions between robots from different fleets, robots and infrastructure, as well as robots and users. In the illustrations below, from top to bottom we have how the world looks like in `traffic_editor`, the schedule visualizer in `rviz`, and the full simulation in `gazebo`,

![](../media/airport_terminal_traffic_editor_screenshot.png)
![](../media/airport_terminal_demo_screenshot.png)

#### Demo Scenario
To launch the world and the schedule visualizer,

```bash
source ~/rmf_ws/install/setup.bash
ros2 launch rmf_demos airport_terminal.launch.xml
```

Select the `airport` tab on RMF Panel. Load the `airport_terminal_tasks.json` list and click submit to begin a collection of loop, delivery and cleaning tasks.

Or, submit `loop`, `delivery` or `clean` task via CLI:
```bash
ros2 run rmf_demos_tasks dispatch_loop -s s07 -f n12 -n 3 --use_sim_time
ros2 run rmf_demos_tasks dispatch_delivery -p mopcart_pickup -pd mopcart_dispenser -d spill -di mopcart_collector --use_sim_time
ros2 run rmf_demos_tasks dispatch_clean -cs zone_3 --use_sim_time
```

To see crowd simulation in action, enable crowd sim by:
```bash
ros2 launch rmf_demos airport_terminal.launch.xml use_crowdsim:=1
```

Non-autonomous vehicles can also be integrated with RMF provided their positions can be localized in the world. This may be of value at facilities where space is shared by autonomous robots as well as manually operated vehicles such as forklifts or transporters. In this demo, we can introduce a vehicle (caddy) which can be driven around through keyboard/joystick teleop. In RMF nomenclature, this vehicle is classified as a `read_only` type, ie, RMF can only infer its position in the world but does not have control over its motion. Here, the goal is to have other controllable robots avoid this vehicle's path by replanning their routes if needed. The model is fitted with a plugin which generates a prediction of the vehicle's path based on its current heading. It is configured to occupy the same lanes as the `tinyRobot` robots. Here, a `read_only_fleet_adapter` submits the prediction from the plugin to the RMF schedule.

In the airport terminal map, a `Caddy` is spawned in the far right corner and can be controlled with `geometry_msgs/Twist` messages published over the `cmd_vel` topic. 

Run `teleop_twist_keyboard` to control the `caddy` with your keyboard:
```bash
# Default launch with gazebo
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# if launch with use_ignition:=1
ros2 launch rmf_demos airport_terminal_caddy_ign.launch.xml
```

![](../media/caddy.gif)

---

### Clinic World

This is a clinic world with two levels and two lifts for the robots. Two different robot fleets with different roles navigate across two levels by lifts. In the illustrations below, we have the view of level 1 in `traffic_editor` (top left), the schedule visualizer in `rviz` (right), and the full simulation in `gazebo` (bottom left).

![](../media/clinic.png)

#### Demo Scenario
To launch the world and the schedule visualizer,

```bash
source ~/rmf_ws/install/setup.bash
ros2 launch rmf_demos clinic.launch.xml
```

Select the `clinic` tab on RMF Panel. Load the `clinic_tasks.json` list and click submit to begin a collection of loop and delivery tasks.

Or, submit a task via CLI:
```bash
ros2 run rmf_demos_tasks dispatch_loop -s L1_left_nurse_center -f L2_right_nurse_center -n 5 --use_sim_time
ros2 run rmf_demos_tasks dispatch_loop -s L2_north_counter -f L1_right_nurse_center -n 5 --use_sim_time
```

Robots taking lift:

![](../media/robot_taking_lift.gif)


Multi-fleet demo:

![](../media/clinic.gif)

---

### Hotel World

This is a hotel with a lobby and a guest level. The hotel has two lifts and two robot fleets. The tiny robots are supposed to guide the guests and the delivery robots are used to load and deliver cargo.

The hotel map is truncated due to the high memory usage. The full map can be accessed [here](https://github.com/MakinoharaShouko/hotel).

Hotel floor plan in `traffic_editor`:
![](../media/hotel.png)

Full hotel floor plan in `traffic_editor`:
![](../media/hotel_full.png)

#### Demo Scenario

To launch the world and the schedule visualizer,

```bash
source ~/rmf_ws/install/setup.bash
ros2 launch rmf_demos hotel.launch.xml
```

Select the `hotel` tab on RMF Panel. Loop requests can be submitted via "Submit a Task" form.

Robot taking lift:

![](../media/robot_taking_lift_hotel.gif)

---

### Traffic Light Robot Demos

RMF can also manage fleets whose API or fleet managers only offer pause and resume commands to control their robots. Such fleets are classified as `traffic_light`. To integrate a `traffic_light` fleet, users are expected to implement a `traffic_light` fleet adapter based on this [API](https://github.com/open-rmf/rmf_ros2/blob/main/rmf_fleet_adapter/include/rmf_fleet_adapter/agv/EasyTrafficLight.hpp). The `rmf_demos` repository contains demonstrations of `traffic_light` fleets in various scenarios. A simplistic `mock_traffic_light` adapter is used in these demonstrations.

#### Triple-H scenario:
```bash
$ ros2 launch rmf_demos triple_H.launch.xml
(new terminal) $ ros2 launch rmf_demos the_pedigree.launch.xml
```
#### Battle Royale Scenario:

```bash
$ ros2 launch rmf_demos battle_royale.launch.xml
(new terminal) $ ros2 launch rmf_demos battle_go.launch.xml
```

#### Office Scenario:
Note that `tinyRobot1` is a standard "full control" robot, while `tinyRobot2` "traffic light" robot.
```bash
$ ros2 launch rmf_demos office_mock_traffic_light.launch.xml
(new terminal) $ ros2 launch rmf_demos office_traffic_light_test.launch.xml
```

## Task Dispatching in RMF
![](../media/RMF_Bidding.png)

In RMF version `21.04` and above, tasks are awarded to robot fleets based on the outcome of a bidding process that is orchestrated by a Dispatcher node, `rmf_dispatcher_node`. When the Dispatcher receives a new task request from a UI, it sends out a `rmf_task_msgs/BidNotice` message to all the fleet adapters. If a fleet adapter is able to process that request, it submits a `rmf_task_msgs/BidProposal` message back to the Dispatcher with a cost to accommodate the task. An instance of `rmf_task::agv::TaskPlanner` is used by the fleet adapters to determine how best to accommodate the new request. The Dispatcher compares all the `BidProposals` received and then submits a `rmf_task_msgs/DispatchRequest` message with the fleet name of the robot that the bid is awarded to. There are a couple different ways the Dispatcher evaluates the proposals such as fastest to finish, lowest cost, etc which can be configured.

Battery recharging is tightly integrated with the new task planner. `ChargeBattery` tasks are optimally injected into a robot's schedule when the robot has insufficient charge to fulfill a series of tasks. Currently we assume each robot in the map has a dedicated charging location as annotated with the `is_charger` option in the traffic editor map.
