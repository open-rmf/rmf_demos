#!/bin/bash

if [ -z "$1" ]
then
      echo "Workspace was not set using default $HOME/rmf_demos_ws"
      WORKSPACE="$HOME/rmf_demos_ws"
else
      WORKSPACE=$1
fi

#Opening Tmux Windows
tmux split-window -h -t 0 -p 50
tmux split-window -v -t 0 -p 20
tmux split-window -v -t 0 -p 25
tmux split-window -v -t 0 -p 33
tmux split-window -v -t 0 -p 50
tmux split-window -v -t 5 -p 16
tmux split-window -v -t 5 -p 19
tmux split-window -v -t 5 -p 24
tmux split-window -v -t 5 -p 37
tmux split-window -v -t 5 -p 50

## Pane 0 ##

#Creating the key artifacts on the keystore

echo "source $WORKSPACE/install/setup.bash
export ROS_SECURITY_KEYSTORE=$WORKSPACE/keystore
export ROS_SECURITY_ENABLE=true
export ROS_SECURITY_STRATEGY=Enforce
export ROS_DOMAIN_ID=42" > $WORKSPACE/sros2_environment.bash
sleep 1

source $WORKSPACE/sros2_environment.bash
ros2 security generate_artifacts -k $WORKSPACE/keystore -p $WORKSPACE/install/rmf_demos/share/rmf_demos/sros2/policies/office.policy.xml
sleep 2

#Switch to CycloneDDS
echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> $WORKSPACE/sros2_environment.bash

## Pane 1 ##

tmux send-keys -t 1 "source $WORKSPACE/sros2_environment.bash" Enter
tmux send-keys -t 1 "ros2 run rmf_traffic_ros2 rmf_traffic_schedule --ros-args --params-file $WORKSPACE/install/rmf_demos/share/rmf_demos/sros2/office.params.yaml --enclave /office/rmf_traffic_schedule_node" Enter
sleep 2

## Pane 2 ##

tmux send-keys -t 2 "source $WORKSPACE/sros2_environment.bash" Enter
tmux send-keys -t 2 "ros2 run rmf_building_map_tools building_map_server $WORKSPACE/install/rmf_demos_maps/share/rmf_demos_maps/office/office.building.yaml --ros-args --enclave /office/building_map_server" Enter
sleep 2

## Pane 3 ##

tmux send-keys -t 3 "source $WORKSPACE/sros2_environment.bash" Enter
tmux send-keys -t 3 "ros2 run rmf_schedule_visualizer rviz2 -r 10 -m L1 --ros-args --params-file $WORKSPACE/install/rmf_demos/share/rmf_demos/sros2/office.params.yaml --enclave /office/rviz2_node" Enter
sleep 2

## Pane 4 ##

tmux send-keys -t 4 "source $WORKSPACE/sros2_environment.bash" Enter
tmux send-keys -t 4 "ros2 run building_systems_visualizer building_systems_visualizer -m L1 --ros-args --params-file $WORKSPACE/install/rmf_demos/share/rmf_demos/sros2/office.params.yaml --enclave /office/building_systems_visualizer" Enter
sleep 2

## Pane 5 ##

tmux send-keys -t 5 "source $WORKSPACE/sros2_environment.bash" Enter
tmux send-keys -t 5 "ros2 run fleet_state_visualizer fleet_state_visualizer -m L1 --ros-args --params-file $WORKSPACE/install/rmf_demos/share/rmf_demos/sros2/office.params.yaml --enclave /office/fleet_state_visualizer" Enter
sleep 2

## Pane 6 ##

tmux send-keys -t 6 "source $WORKSPACE/sros2_environment.bash" Enter
tmux send-keys -t 6 "rviz2 -d $WORKSPACE/install/rmf_demos/share/rmf_demos/include/office/office.rviz --ros-args --enclave /office/rviz2" Enter
sleep 2

## Pane 7 ##

tmux send-keys -t 7 "source $WORKSPACE/sros2_environment.bash" Enter
tmux send-keys -t 7 "ros2 run rmf_fleet_adapter door_supervisor --ros-args --enclave /office/door_supervisor" Enter
sleep 2

## Pane 8 ##

echo "export GAZEBO_MODEL_PATH=$WORKSPACE/install/rmf_demos_maps/share/rmf_demos_maps/maps/office/models:$WORKSPACE/install/rmf_demos_assets/share/rmf_demos_assets/models:/usr/share/gazebo-11/models
export GAZEBO_RESOURCE_PATH=$WORKSPACE/install/rmf_demos_assets/share/rmf_demos_assets:/usr/share/gazebo-11
export GAZEBO_PLUGIN_PATH=$WORKSPACE/install/rmf_robot_sim_gz_classic_plugins/lib:$WORKSPACE/install/rmf_building_sim_gz_classic_plugins/lib/
export GAZEBO_MODEL_DATABASE_URI=\"\"" > $WORKSPACE/gazebo_environment.bash

tmux send-keys -t 8 "source $WORKSPACE/sros2_environment.bash" Enter
tmux send-keys -t 8 "source $WORKSPACE/gazebo_environment.bash" Enter
tmux send-keys -t 8 "gzserver --verbose -s libgazebo_ros_factory.so -s libgazebo_ros_init.so $WORKSPACE/install/rmf_demos_maps/share/rmf_demos_maps/maps/office/office.world --ros-args --enclave /office/gzserver" Enter
sleep 2

## Pane 9 ##

tmux send-keys -t 9 "source $WORKSPACE/gazebo_environment.bash" Enter
tmux send-keys -t 9 "gzclient --verbose $WORKSPACE/install/rmf_demos_maps/share/rmf_demos_maps/maps/office/office.world" Enter
sleep 2

## Pane 10 ##

tmux send-keys -t 10 "cd $WORKSPACE" Enter
tmux send-keys -t 10 "source $WORKSPACE/sros2_environment.bash" Enter
tmux send-keys -t 10 "ros2 run rmf_fleet_adapter full_control --ros-args -r __node:=tinyRobot_fleet_adapter --params-file $WORKSPACE/install/rmf_demos/share/rmf_demos/sros2/office.params.yaml --enclave /office/tinyRobot_fleet_adapter" Enter
sleep 2

## Pane 0 ##

source $WORKSPACE/sros2_environment.bash
ros2 run rmf_fleet_adapter robot_state_aggregator --ros-args -r __node:=tinyRobot_state_aggregator --params-file $WORKSPACE/install/rmf_demos/share/rmf_demos/sros2/office.params.yaml --enclave /office/tinyRobot_state_aggregator
