#!/usr/bin/env python3

# Copyright 2022 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import launch
import launch_ros
import lifecycle_msgs.msg

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


# supply args: map_name, transform [tx, ty, yaw]
# if use_launch_config, this means the input param is from cli launch arg
def create_map_server(
    ld,
    map_name,
    tx,
    ty,
    yaw,
    world_name,
    use_launch_config=False,
):
    # static pub mode
    tf_pub_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name=f"static_transform_publisher",
        namespace=map_name,
        arguments=[tx, ty, '0', yaw, '0', '0', 'map', map_name])

    pkg_share_dir = get_package_share_directory('rmf_demos_maps')

    if use_launch_config:
        yaml_filename = \
            [pkg_share_dir, "/", world_name, "/", map_name, ".yaml"]
    else:
        yaml_filename = f"{pkg_share_dir}/{world_name}/{map_name}.yaml"

    # map server node
    map_server_node = launch_ros.actions.LifecycleNode(
        package="nav2_map_server",
        executable="map_server",
        name=f"map_server",
        namespace=map_name,
        output="screen",
        emulate_tty=True,
        parameters=[
            {"yaml_filename": yaml_filename},
            {"frame_id": map_name},
            {"topic_name": "map"},
        ],
        )

    configure_trans_event = launch.actions.EmitEvent(
        event=launch_ros.events.lifecycle.ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(
                map_server_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    activate_trans_event = launch.actions.EmitEvent(
        event=launch_ros.events.lifecycle.ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(
                map_server_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
        )
    )

    # Add the actions to the launch description.
    # The order should reflects the order in which they will be executed.
    ld.add_action(tf_pub_node)
    ld.add_action(map_server_node)
    ld.add_action(configure_trans_event)
    ld.add_action(activate_trans_event)
    return ld


def generate_launch_description():

    map_name = LaunchConfiguration('map_name')
    tx = LaunchConfiguration('tx')
    ty = LaunchConfiguration('ty')
    yaw = LaunchConfiguration('yaw')
    world_name = LaunchConfiguration('world_name')

    declare_map_name_cmd = DeclareLaunchArgument(
        'map_name',
        default_value='',
        description='map name, should also corespond to the .yaml file name',
    )
    declare_tx_cmd = DeclareLaunchArgument(
        'tx', default_value='0.0', description='x-axis translation')
    declare_ty_cmd = DeclareLaunchArgument(
        'ty', default_value='0.0', description='y-axis translation')
    declare_yaw_cmd = DeclareLaunchArgument(
        'yaw', default_value='0.0', description='yaw angle in radian')
    declare_tx_cmd = DeclareLaunchArgument(
        'world_name', default_value='office', description='name of the world')

    ld = launch.LaunchDescription()
    ld.add_action(declare_map_name_cmd)
    ld.add_action(declare_tx_cmd)
    ld.add_action(declare_ty_cmd)
    ld.add_action(declare_yaw_cmd)

    # With this, can be launched with cli:
    #    ros2 launch rmf_demos map_server.launch.py \
    #        map_name:=ecobot_office tx:=1.33 ty:=0.057 yaw:=-1.598
    # Note: transformation value is obtained from traffic-editor
    ld = create_map_server(
        ld, map_name, tx, ty, yaw, world_name, use_launch_config=True)
    return ld
