#!/bin/bash

echo "Starting basic loop and patrolling scenario."

function spawn_robot {
  ros2 run gazebo_ros spawn_entity.py \
    -database $1 \
    -reference_frame $2_placeholder \
    -entity $2 \
    -z 0.05
}

echo "Spawning robot deliveryRobot_0."
spawn_robot DeliveryRobot deliveryRobot_0

echo "Spawning robot deliveryRobot_2."
spawn_robot DeliveryRobot deliveryRobot_2

echo "Spawning robot tinyRobot_0."
spawn_robot TinyRobot tinyRobot_0

echo "Spawning robot tinyRobot_1."
spawn_robot TinyRobot tinyRobot_1

echo "Spawning robot tinyRobot_2."
spawn_robot TinyRobot tinyRobot_2

echo "Spawning robot tinyRobot_4."
spawn_robot TinyRobot tinyRobot_4
