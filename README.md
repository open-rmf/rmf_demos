# RMF Demos

![](https://github.com/open-rmf/rmf_demos/workflows/build/badge.svg)
![](https://github.com/open-rmf/rmf_demos/workflows/style/badge.svg)

The Open Robotics Middleware Framework (Open-RMF) enables interoperability among heterogeneous robot fleets while managing robot traffic that share resources such as space, building infrastructure systems (lifts, doors, etc) and other automation systems within the same facility. Open-RMF also handles task allocation and conflict resolution  among its participants (de-conflicting traffic lanes and other resources). These capabilities are provided by various libraries in [Open-RMF](https://github.com/open-rmf/rmf).
For more details about Open RMF, refer to the comprehensive documentation provided [here](https://osrf.github.io/ros2multirobotbook/intro.html).

This repository contains demonstrations of the above mentioned capabilities of RMF. It serves as a starting point for working and integrating with Open-RMF.

You can also find a nice demonstration of Open-RMF using `Nav2` and `MoveIt!` built into the [Ionic Release Demo](https://github.com/gazebosim/ionic_demo).

[![Robotics Middleware Framework](../media/thumbnail.png?raw=true)](https://vimeo.com/405803151)

#### (Click to watch video)

## System Requirements

These demos were built and tested on

* [Ubuntu 24.04 LTS](https://releases.ubuntu.com/24.04/)

* [ROS 2 - Kilted](https://docs.ros.org/en/jazzy/Releases/Release-Kilted-Kaiju.html)

* [Gazebo Ionic](https://gazebosim.org/docs/ionic)
> Note: The `main` branches of the core RMF libraries are fully supported on ROS 2 Humble, Iron, and Jazzy as well, but you will need to use the distro-specific branches for `rmf_traffic_editor` and `rmf_simulation`.
>

## Installation
Instructions can be found [here](https://github.com/open-rmf/rmf).

## FAQ
Answers to frequently asked questions can be found [here](docs/faq.md).

## Roadmap

A near-term roadmap of the Open-RMF project can be found in the user manual [here](https://osrf.github.io/ros2multirobotbook/roadmap.html).

## RMF-Web quick start

Full web application of Open-RMF: [rmf-web](https://github.com/open-rmf/rmf-web).

Start the backend API server via `docker` with host network access, using the default configuration. The API server will be accessible at `localhost:8000` by default.

```bash
docker run \
  --network host \
  -it \
  -e ROS_DOMAIN_ID=<ROS_DOMAIN_ID> \
  -e RMW_IMPLEMENTATION=<RMW_IMPLEMENTATION> \
  ghcr.io/open-rmf/rmf-web/api-server:latest
```

> Note: The API server is also configurable by mounting the configuration file and setting the environment variable `RMF_API_SERVER_CONFIG`. In the default configuration, the API serer will use an internal non-persistent database.

Start the frontend dashboard via `docker` with host network access, using the default configuration. The dashboard will be accessible at `localhost:3000` by default.

```bash
docker run \
  --network host \
  -it \
  ghcr.io/open-rmf/rmf-web/dashboard:latest
```

> Note: The dashboard via `docker` is not runtime-configurable and is best used for quick integrations and testing. To configure the dashboard, check out [rmf-web-dashboard-resources](https://github.com/open-rmf/rmf_demos/tree/rmf-web-dashboard-resources/rmf_demos_dashboard_resources) and the [dashboard configuration section](https://github.com/open-rmf/rmf-web/tree/main/packages/dashboard#configuration).

In order to interact with the default configuration of the web application, the `server_uri` launch parameter will need to be changed to `ws://localhost:8000/_internal`, for example,

```bash
ros2 launch rmf_demos_gz office.launch.xml server_uri:="ws://localhost:8000/_internal"
```

By specifying `server_uri`, the fleetadapter will update `rmf-web` `api-server` with the latest task and robot states. User can then monitor on-going states and initiate rmf task with an interactive web dashboard.

## Demo Worlds

* [Hotel World](#Hotel-World)
* [Office World](#Office-World)
* [Airport Terminal World](#Airport-Terminal-World)
* [Clinic World](#Clinic-World)
* [Campus World](#Campus-World)
* [Manufacturing & Logistics World](#Manufacturing-&-Logistics-World)

---

### Hotel World

This hotel world consists of a lobby and 2 guest levels. The hotel has two lifts, multiple doors and 3 robot fleets (4 robots).
This demonstrates an integration of multiple fleets of robots with varying capabilities working together in a multi-level building.

![](../media/hotel_world.png)

#### Demo Scenario

To launch the world and the schedule visualizer,

```bash
source ~/rmf_ws/install/setup.bash
ros2 launch rmf_demos_gz hotel.launch.xml

# Or, run with ignition simulator
ros2 launch rmf_demos_gz hotel.launch.xml
```

Here, we will showcase 2 types of Tasks: **Loop** and **Clean**, you can dispatch them via CLI as follows:
```bash
ros2 run rmf_demos_tasks dispatch_patrol -p restaurant  L3_master_suite -n 1 --use_sim_time
ros2 run rmf_demos_tasks dispatch_clean -cs clean_lobby --use_sim_time
```

Robots running Clean and Loop Task:

![](../media/hotel_scenarios.gif)

---

### Office World
An indoor office environment for robots to navigate around. It includes a beverage dispensing station, controllable doors and laneways which are integrated into RMF.

```bash
source ~/rmf_demos_ws/install/setup.bash
ros2 launch rmf_demos_gz office.launch.xml

# Or, run with ignition simulator
ros2 launch rmf_demos_gz office.launch.xml
```

Now we will showcase 2 types of Tasks: **Delivery** and **Loop**

![](../media/delivery_request.gif?raw=true)

You can request the robot to deliver a can of coke from `pantry` to `hardware_2` through the following:
```bash
ros2 run rmf_demos_tasks dispatch_delivery -p pantry -ph coke_dispenser -d hardware_2 -dh coke_ingestor --use_sim_time
```

You can also request the robot to move back and forth between `coe` and `lounge` through the following:
```bash
ros2 run rmf_demos_tasks dispatch_patrol -p coe lounge -n 3 --use_sim_time
```

![](../media/loop_request.gif)

The office demo can be run in secure mode using ROS 2 DDS-Security integration. Click [here](docs/secure_office_world.md) to learn more.

---

### Airport Terminal World

This demo world shows robot interaction on a much larger map, with a lot more lanes, destinations, robots and possible interactions between robots from different fleets, robots and infrastructure, as well as robots and users. In the illustrations below, from top to bottom we have how the world looks like in `traffic_editor`, the schedule visualizer in `rviz`, and the full simulation in `gazebo`,

![](../media/airport_terminal_traffic_editor_screenshot.png)
![](../media/airport_terminal_demo_screenshot.png)

#### Demo Scenario
In the airport world, we introduce a new task type to rmf: `Clean`. To launch the world:

```bash
source ~/rmf_ws/install/setup.bash
ros2 launch rmf_demos_gz airport_terminal.launch.xml
```

You can submit `loop`, `delivery` or `clean` task via CLI:
```bash
ros2 run rmf_demos_tasks dispatch_patrol -p s07 n12 -n 3 --use_sim_time
ros2 run rmf_demos_tasks dispatch_delivery -p mopcart_pickup -ph mopcart_dispenser -d spill -dh mopcart_collector --use_sim_time
ros2 run rmf_demos_tasks dispatch_clean -cs zone_3 --use_sim_time
```

To see crowd simulation in action, enable crowd sim by:
```bash
ros2 launch rmf_demos_gz airport_terminal.launch.xml use_crowdsim:=1
```

Non-autonomous vehicles can also be integrated with Open-RMF provided their positions can be localized in the world. This may be of value at facilities where space is shared by autonomous robots as well as manually operated vehicles such as forklifts or transporters. In this demo, we can introduce a vehicle (caddy) which can be driven around through keyboard/joystick teleop. In Open-RMF nomenclature, this vehicle is classified as a `read_only` type, ie, Open-RMF can only infer its position in the world but does not have control over its motion. Here, the goal is to have other controllable robots avoid this vehicle's path by replanning their routes if needed. The model is fitted with a plugin which generates a prediction of the vehicle's path based on its current heading. It is configured to occupy the same lanes as the `tinyRobot` robots. Here, a `read_only_fleet_adapter` submits the prediction from the plugin to the Open-RMF schedule.

In the airport terminal map, a `Caddy` is spawned in the far right corner and can be controlled with `geometry_msgs/Twist` messages published over the `cmd_vel` topic.

Run `teleop_twist_keyboard` to control the `caddy` with your keyboard:
```bash
# Default launch with gazebo
ros2 run teleop_twist_keyboard teleop_twist_keyboard

ros2 launch rmf_demos_gz airport_terminal_caddy.launch.xml
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
ros2 launch rmf_demos_gz clinic.launch.xml
```

You can submit tasks via CLI:
```bash
ros2 run rmf_demos_tasks dispatch_patrol -p L1_left_nurse_center L2_right_nurse_center -n 5 --use_sim_time
ros2 run rmf_demos_tasks dispatch_patrol -p L2_north_counter L1_right_nurse_center -n 5 --use_sim_time
```

Robots taking lift:

![](../media/robot_taking_lift.gif)


Multi-fleet demo:

![](../media/clinic.gif)

---
### Campus World

This is a larger scale "Campus" World. In this world, there are multiple delivery robots that operate. The world is designed and traffic lanes are annotated at the planet scale, using GPS WGS84 coordinates. Each robot is also streaming its location in WGS84 coordinates, which are processed by its fleet adapter. This demo intends to show the potential of Open-RMF on a large scale map.

![](../media/campus.gif)

#### Demo Scenario
To launch the world and the schedule visualizer,

```bash
source ~/rmf_ws/install/setup.bash
ros2 launch rmf_demos_gz campus.launch.xml

ros2 run rmf_demos_tasks  dispatch_patrol -p room_5 campus_4 -n 10 --use_sim_time
ros2 run rmf_demos_tasks  dispatch_patrol -p campus_5 room_3 -n 10 --use_sim_time
ros2 run rmf_demos_tasks  dispatch_patrol -p room_2 dead_end -n 10 --use_sim_time
```

#### RobotManager Integration
`fleet_robotmanager_mqtt_bridge` (see [rmf_demos_bridges](https://github.com/open-rmf/rmf_demos/tree/main/rmf_demos_bridges/rmf_demos_bridges)) can be used to publish robot locations, battery percentage and state to a `/robot/status/ROBOT-ID` websocket endpoint. An instance of RobotManager can be configured to subscribe to this server to receive json messages, which will in turn visualize the robots on RobotManager.

```bash
# Install the prerequisites
sudo apt install mosquitto mosquitto-clients

# Start the bridge
ros2 run rmf_demos_bridges fleet_robotmanager_mqtt_bridge -y 31500 -x 22000
```

The json messages for the first robot can be echoed using the following example command,

```bash
mosquitto_sub -t /robot/status/00000000-0000-0000-0000-000000000001
```

---
### Manufacturing & Logistics World

An Open-RMF simulation demonstration created by ROS-Industrial Asia Pacific showcasing workcell (conveyor and fixed manipulator), multiple AMR fleets and infrastructure interoperability using the Open Robotics Middleeware Framework (Open-RMF).

<p align="center">
[![Alt text](https://img.youtube.com/vi/oSVQrjx_4w4/0.jpg)](https://www.youtube.com/watch?v=oSVQrjx_4w4)
</p>

## Other Tools and Features Demos

* [Traffic Light Robot Demos](#Traffic-Light-Robot-Demos)
* [Additional Features](#Additional-Features)
* [Task Dispatching in Open-RMF](#Task-Dispatching-in-Open-RMF)

### Traffic Light Robot Demos

Open-RMF can also manage fleets whose API or fleet managers only offer pause and resume commands to control their robots. Such fleets are classified as `traffic_light`. To integrate a `traffic_light` fleet, users are expected to implement a `traffic_light` fleet adapter based on this [API](https://github.com/open-rmf/rmf_ros2/blob/main/rmf_fleet_adapter/include/rmf_fleet_adapter/agv/EasyTrafficLight.hpp). The `rmf_demos` repository contains demonstrations of `traffic_light` fleets in various scenarios. A simplistic `mock_traffic_light` adapter is used in these demonstrations.

#### Triple-H scenario:
```bash
$ ros2 launch rmf_demos_gz triple_H.launch.xml
(new terminal) $ ros2 launch rmf_demos the_pedigree.launch.xml
```
#### Battle Royale Scenario:

```bash
$ ros2 launch rmf_demos_gz battle_royale.launch.xml
(new terminal) $ ros2 launch rmf_demos battle_go.launch.xml
```

#### Office Scenario:
Note that `tinyRobot1` is a standard "full control" robot, while `tinyRobot2` "traffic light" robot.
```bash
$ ros2 launch rmf_demos_gz office_mock_traffic_light.launch.xml
(new terminal) $ ros2 launch rmf_demos office_traffic_light_test.launch.xml
```

### Additional Features
 - **Flexible Tasks Scripts**
   For more [details](rmf_demos_tasks/README.md).

 - **lift watchdog**
   - The robot can query an external `lift_watchdog_server` for the permission to enter the lift cabin during the `LiftSession` Phase.
   - Command lines:
    ```bash
    # run hotel world with lift_watch_dog enabled
    ros2 launch rmf_demos_gz hotel.launch.xml enable_experimental_lift_watchdog:=1

    ## On a separate terminal, set lift as crowded
    ros2 launch rmf_demos experimental_crowded_lift.launch.xml

    # Dispatch robot from level1 to level3, robot will wait in front of the lift cabin
    ros2 run rmf_demos_tasks dispatch_patrol -p L3_room1  L3_room1 -n 1 --use_sim_time

    # Lift is cleared. Give robot the permission to enter the lift
    ros2 launch rmf_demos experimental_clear_lift.launch.xml
    ```
 - **Custom Docking Sequence**
    - Fleet adapter will notify the robot (via `dock()` api/ModeRequest) to execute its custom dock sequence when the robot reaches a "dock" waypoint.
    - Implementation is similar to Clean task, refer to docs [here](https://osrf.github.io/ros2multirobotbook/task_types.html?highlight=docking#step-1-defining-waypoints-for-cleaning-in-traffic-editor)

 - **Emergency Alarm**
   - All robots will get directed to the nearest parking spot when the emergency alarm is triggered.
   - Command lines:
    ```bash
    # toggle alarm ON
    ros2 topic pub -1 /fire_alarm_trigger std_msgs/Bool '{data: true}'

    # toggle alarm OFF
    ros2 topic pub -1 /fire_alarm_trigger std_msgs/Bool '{data: false}'
    ```

## Task Dispatching in Open-RMF
![](../media/RMF_Bidding.png)

In Open-RMF version `21.04` and above, tasks are awarded to robot fleets based on the outcome of a bidding process that is orchestrated by a Dispatcher node, `rmf_dispatcher_node`. When the Dispatcher receives a new task request from a UI, it sends out a `rmf_task_msgs/BidNotice` message to all the fleet adapters. If a fleet adapter is able to process that request, it submits a `rmf_task_msgs/BidProposal` message back to the Dispatcher with a cost to accommodate the task. An instance of `rmf_task::agv::TaskPlanner` is used by the fleet adapters to determine how best to accommodate the new request. The Dispatcher compares all the `BidProposals` received and then submits a `rmf_task_msgs/DispatchRequest` message with the fleet name of the robot that the bid is awarded to. There are a couple different ways the Dispatcher evaluates the proposals such as fastest to finish, lowest cost, etc which can be configured.

Battery recharging is tightly integrated with the new task planner. `ChargeBattery` tasks are optimally injected into a robot's schedule when the robot has insufficient charge to fulfill a series of tasks. Currently we assume each robot in the map has a dedicated charging location as annotated with the `is_charger` option in the traffic editor map.
