#!/bin/bash

echo "Starting delivery scenario."

function spawn_robot {
  ros2 run gazebo_ros spawn_entity.py \
    -database $1 \
    -reference_frame $2_placeholder \
    -entity $2 \
    -z 0.05
}

function send_delivery_job {
  ros2 run rmf_demo_tasks request_delivery \
    --pickup $1 \
    --dropoff $2 \
    --robot-type $3
}

echo "Spawning and sending job to robot magni_4."
spawn_robot Magni magni_4
send_delivery_job mopcart_pickup spill magni
