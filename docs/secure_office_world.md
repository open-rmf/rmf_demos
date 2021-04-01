#### Secure ROS 2 Office Demo

The office demo can be run in secure mode using the [ROS 2 DDS-Security integration](https://design.ros2.org/articles/ros2_dds_security.html) (SROS2) capabilities. This security features will provide encryption, authentication and access control to the whole ROS2 setup of RMF.

Because of the heavy development undergoing RMF, changes happen in a very fast manner. Therefore, the ROS 2 Access Control Policies declared in the `office.policy.xml` file are only guaranteed to work with the RMF release 1.1.X, so please set all the git repos of your RMF workspace to point to the tag `1.1.0`. Alternatively you could update the policies yourself to any later version of RMF (PRs are welcome if you do so :smirk:).

There is a `tmux` based script that automates pane splitting and the execution of all the required commands to run the office demo. In order to run it this way just open a tmux terminal and run the script.

```
bash ./install/rmf_demos/share/rmf_demos/sros2/office_deploy.bash
```

Alongside, the following steps can be used to run all required commands individually.

To use ROS 2 to secure the office demo a few environmental variables need to be set. Adding them to a script would make it easier to source them in any shell where we need to run ROS 2 secured nodes.

```bash
echo 'export ROS_SECURITY_KEYSTORE=~/rmf_demos_ws/keystore
export ROS_SECURITY_ENABLE=true
export ROS_SECURITY_STRATEGY=Enforce
export ROS_DOMAIN_ID=42' > sros2_environment.sh
```

A few security artifacts like signed permission files, identity certificates and Certificate Authorities (CA) are needed to enable the DDS Security. They can be generated using the security tools from the ROS2 cli. Because of [#242](https://github.com/ros2/sros2/issues/242) and until its solution [#238](https://github.com/ros2/sros2/pull/238) is released it is recommended to generate the security artifacts before switching to `cyclone_dds`.

```
source sros2_environment.sh
mkdir keystore
ros2 security generate_artifacts -k keystore -p ./install/rmf_demos/share/rmf_demos/sros2/policies/office.policy.xml
```

It is recommended to use `cyclone dds` with the secure version of the demo, make sure it is properly installed, with security support and selected through the correspondent environment variable ([here](https://index.ros.org/doc/ros2/Installation/DDS-Implementations/Working-with-Eclipse-CycloneDDS/) you can find instructions on how to do it). Adding this to the environment script makes it easier to source in any terminal that might need it,

```bash
echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> sros2_environment.sh
```

Because SROS2 does not support `ros2 launch` yet all the different nodes will have to be launched individually. A `yaml` file with the parameters of all the nodes is provided. One terminal per execution is recommended for an easier debug of the system. Make sure you `source ~/rmf_demos_ws/install/setup.bash` in each of them before executing the different commands.

```bash
source sros2_environment.sh
ros2 run rmf_traffic_ros2 rmf_traffic_schedule --ros-args --params-file ./install/rmf_demos/share/rmf_demos/sros2/office.params.yaml --enclave /office/rmf_traffic_schedule_node
```

```bash
source sros2_environment.sh
ros2 run rmf_building_map_tools building_map_server ./install/rmf_demos_maps/share/rmf_demos_maps/office/office.building.yaml --ros-args --enclave /office/building_map_server
```

```bash
source sros2_environment.sh
ros2 run rmf_schedule_visualizer rviz2 -r 10 -m L1 --ros-args --params-file ./install/rmf_demos/share/rmf_demos/sros2/office.params.yaml --enclave /office/rviz2_node
```

```bash
source sros2_environment.sh
ros2 run building_systems_visualizer building_systems_visualizer -m L1 --ros-args --params-file ./install/rmf_demos/share/rmf_demos/sros2/office.params.yaml --enclave /office/building_systems_visualizer
```

```bash
source sros2_environment.sh
ros2 run fleet_state_visualizer fleet_state_visualizer -m L1 --ros-args --params-file ./install/rmf_demos/share/rmf_demos/sros2/office.params.yaml --enclave /office/fleet_state_visualizer
```

```bash
source sros2_environment.sh
rviz2 -d ./install/rmf_demos/share/rmf_demos/include/office/office.rviz --ros-args --enclave /office/rviz2
```

```bash
source sros2_environment.sh
ros2 run rmf_fleet_adapter door_supervisor --ros-args --enclave /office/door_supervisor
```

Gazebo needs the environment variables to locate files, and set up communications between the server and clients. They can be added to a script for an easier re-use.

```bash
echo 'export GAZEBO_MODEL_PATH=~/rmf_demos_ws/install/rmf_demos_maps/share/rmf_demos_maps/maps/office/models:~/rmf_demos_ws/install/rmf_demos_assets/share/rmf_demos_assets/models:/usr/share/gazebo-11/models
export GAZEBO_RESOURCE_PATH=~/rmf_demos_ws/install/rmf_demos_assets/share/rmf_demos_assets:/usr/share/gazebo-11
export GAZEBO_PLUGIN_PATH=~/rmf_demos_ws/install/rmf_robot_sim_gazebo_plugins/lib:~/rmf_demos_ws/install/building_gazebo_plugins/lib/
export GAZEBO_MODEL_DATABASE_URI=""' > gazebo_environment.sh
```

```bash
source sros2_environment.sh
source gazebo_environment.sh
gzserver --verbose -s libgazebo_ros_factory.so -s libgazebo_ros_init.so ./install/rmf_demos_maps/share/rmf_demos_maps/maps/office/office.world --ros-args --enclave /office/gzserver
```

Because of [#151](https://github.com/osrf/rmf_rmf_demos/issues/151), the ros node created by plugins run on gzclient will be left out of the secured network. This should only affect the `toggle_floors` plugin that runs on gzclient and is intended for multilevel demos.

```bash
source gazebo_environment.sh
gzclient --verbose ./install/rmf_demos_maps/share/rmf_demos_maps/maps/office/office.world
```

```bash
source sros2_environment.sh
ros2 run rmf_fleet_adapter full_control --ros-args -r __node:=tinyRobot_fleet_adapter --params-file ./install/rmf_demos/share/rmf_demos/sros2/office.params.yaml --enclave /office/tinyRobot_fleet_adapter
```

```bash
source sros2_environment.sh
ros2 run rmf_fleet_adapter robot_state_aggregator --ros-args -r __node:=tinyRobot_state_aggregator --params-file ./install/rmf_demos/share/rmf_demos/sros2/office.params.yaml --enclave /office/tinyRobot_state_aggregator
```

The system is now ready to attend simulated delivery requests as per usual but with all the guarantees of [DDS-security](https://www.omg.org/spec/DDS-SECURITY/1.1/PDF)!