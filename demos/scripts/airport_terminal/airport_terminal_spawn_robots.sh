#!/bin/bash

echo "Starting basic loop and patrolling scenario."

function spawn_robot {
  ros2 run gazebo_ros spawn_entity.py \
    -database $1 \
    -reference_frame $2_placeholder \
    -entity $2 \
    -z 0.05
}

echo "Spawning robot mir100_0."
spawn_robot MiR100 mir100_0

echo "Spawning robot mir100_2."
spawn_robot MiR100 mir100_2

echo "Spawning robot magni_0."
spawn_robot Magni magni_0

echo "Spawning robot magni_1."
spawn_robot Magni magni_1

echo "Spawning robot magni_2."
spawn_robot Magni magni_2

echo "Spawning robot magni_4."
spawn_robot Magni magni_4
