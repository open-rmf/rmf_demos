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
echo "Spawning and sending job to robot deliveryRobot_1."
spawn_robot DeliveryRobot deliveryRobot_0
send_loop_job junction_north_west junction_south_west deliveryRobot

echo "Spawning and sending job to robot deliveryRobot_2."
spawn_robot DeliveryRobot deliveryRobot_2

echo "Spawning and sending job to robot tinyRobot_0."
spawn_robot TinyRobot tinyRobot_0
send_loop_job tinyRobot_n09 tinyRobot_s07 tinyRobot

echo "Spawning and sending job to robot tinyRobot_1."
spawn_robot TinyRobot tinyRobot_1
send_loop_job tinyRobot_n11 tinyRobot_s07 tinyRobot

# spawn only
echo "Spawning and sending job to robot tinyRobot_2."
spawn_robot TinyRobot tinyRobot_2

echo "Spawning and sending job to robot tinyRobot_4."
spawn_robot TinyRobot tinyRobot_4
