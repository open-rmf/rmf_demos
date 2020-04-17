#!/bin/bash

echo "Starting basic loop and patrolling scenario."

function spawn_robot {
  ros2 run gazebo_ros spawn_entity.py \
    -database $1 \
    -reference_frame $2_placeholder \
    -entity $2 \
    -z 0.05
}

function send_loop_job {
  ros2 run rmf_demo_tasks delayed_request_loop \
    -s $1 \
    -f $2 \
    -n 100 \
    -r $3 \
    --delay 2
}

# spawn and send loop requests
echo "Spawning and sending job to robot mir100_1."
spawn_robot MiR100 mir100_0
send_loop_job junction_north_west junction_south_west mir100

echo "Spawning and sending job to robot mir100_2."
spawn_robot MiR100 mir100_2

echo "Spawning and sending job to robot magni_0."
spawn_robot Magni magni_0
send_loop_job magni_n09 magni_s07 magni

echo "Spawning and sending job to robot magni_1."
spawn_robot Magni magni_1
send_loop_job magni_n11 magni_s07 magni

# spawn only
echo "Spawning and sending job to robot magni_2."
spawn_robot Magni magni_2

echo "Spawning and sending job to robot magni_4."
spawn_robot Magni magni_4
