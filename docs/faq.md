# Frequently Asked Questions

Greetings,

This page aims to document frequently asked questions pertaining to the demonstrations of the Robotics Middleware Framework (also termed RoMi-H) as showcased in this repository. The `rmf_core` repository has an additional [FAQ page](https://github.com/open-rmf/rmf_ros2/blob/master/docs/faq.md) which addresses some important higher level topics.


#### Gazebo crashes with `No namespace found` error at launch

This error is known to arise when launching a demo with `Gazebo 9.0.0`. This is the default version that is come with the bionic release of Ubuntu. The demos however require `Gazebo 9.12.0` or `Gazebo 9.13.0`. To update from `Gazebo 9.0.0`, first make sure your package source is up to date.

```
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list
wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
```
Then run
```
sudo apt-get update
sudo apt-get upgrade
```


#### Gazebo fails to load several models when launching the Airport Terminal world

The Airport Terminal world is populated with several models that are hosted [here](https://github.com/osrf/gazebo_models) and hence not included in `rmf_demo_assets` to avoid maintaining duplicates. Individual model folders may be downloaded into `~/.gazebo/models/` or the entire collection with the commands below.

```bash
cd ~/.
git clone https://github.com/osrf/gazebo_models
cd gazebo_models
cp -r ./* ~/.gazebo/models/.
```
Now when the demo is re-launched, Gazebo will be able to find the required models.


#### Add/Modify models in the Gazebo World
The position and orientation of models is defined using the `traffic_editor`. Load the desired `.project.yaml` file into `traffic_editor` to add or modify models into the floor plan.


#### Running the simulation with Ignition
Work on supporting the simulations in Ignition is underway. The setup instructions will be updated in the near future. 


#### Does RMF work with robots running navigation stacks? The robots in the demo only travel in straight line segments.

Yes. RMF works independent of the technique used by the robot to localize itself and navigate through coordinates in a map. This is because RMF deals with planning at a higher, traffic level which considers the presence of other robot fleets in the same space. RMF maintains a `schedule database` of the intended paths all robots operating in the same environment and monitors the same for potential conflicts that may arise. The costmap is not an occupancy grid of sensor-detected obstacles but rather a database of spaciotemporal representation of robot trajectories. These trajectories are interpolated as cubic splines and account for the respective robot's kinematics.

A trajectory is submitted to the `schedule database` by an `rmf_fleet_adapter` which interfaces with the proprietary `fleet manager` or robot API. The `rmf_fleet adapters` are configurable depending on the level of control offered by the vendor robot fleet. In general a `rmf_fleet_adapter` keeps track of the trajectories of its robots and computes new plans, when a conflict is detected, by entering into a negotiation with the conflicting robot's `rmf_fleet_adapter`. This is possible as each `rmf_fleet_adapter` maintains a mirror of the `schedule database` and hence is aware all the trajectories active in the space. The generated plan may ask the robot to wait at a node or reroute itself along other lanes in the graph. The result of the plan is communicated to the robot's `fleet manager` via a `PathRequest` message. The expectation is that the vendor `fleet manager` will comply to the incoming request and command the respective robot to reach follow the waypoints in the `PathRequest` message to reach its destination, conflict free. Here the robot will make use of its navigation stack and drivers to drive to each of the waypoints while avoiding sensor-detected obstacles. In the meanwhile, the `rmf_fleet_adapter` expects the `fleet manager` to update the current position of the robot and its remaining path back to it via a `RobotState` message. Then for example, if a robot pauses due to a dynamic obstacle in its path, the information is implicitly conveyed via the `RobotState` message and this delay is submitted to the `schedule database` which is now visible to the other `rmf_fleet_adapters`, and can inturn replan their robot routes if affected by the delay.

To replicate this interaction between `rmf_fleet_adapters` and vendor robots in simulation, we have the `slotcar` plugin. This plugin does two things 1) Interface with an `rmf_fleet_adapter` via the described messages and 2) Navigate the robot to the requested waypoint by actuating the model joints. For the later, it implements a simple "rail-like" navigation which accelerates and decelerates the robot along a straight line from its current position to the waypoint. We assume that the robot lanes are free of other static obstacles. This is implemented to minimize computational load on systems running the demos which tends to become very significant when running a ROS1/2 navstack for every robot in the simulation. Since the focus here is on traffic management of heterogeneous fleets and not robot navigation, the `slotcar` plugins allow us to efficiently test various scenarios in simulation.

However, it certainly possible to run the demos with robots powered by navstacks. This would require running a separate node which does 1) Send and receive `RobotState` and `PathRequest` messages resp. as did the `slotcar` and 2) Send goals to the navstack's action server corresponding to the waypoints received in the `PathRequest` message and update the robot's state along the way. Transformations (rotation, translation or scaling) if necessary may be applied to the goal coordinates to account for any variations between the onboard `.pgm` map and the floor plan annotated by the `traffic_editor`. These can be checked for by importing the `.pgm` file into `traffic_editor`.
This is what we do when we test RMF on physical robots.


#### How does RMF detect conflicts and replan routes for mobile robots?
Kindly refer to the [FAQ](https://github.com/open-rmf/rmf_ros2/blob/master/docs/faq.md#how-does-rmf_traffic-avoid-mobile-robot-traffic-conflicts) in `rmf_core`


#### Has RMF been tested with physical robots?
Yes, RMF has been tested with multiple brands of robots (ROS and non-ROS based) operating in the same space together. Among these robots, two variants offer `full control` integration while the other, `read_only`. More information on integration will be documented in the near future.
