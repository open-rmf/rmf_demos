#!/usr/bin/env python3

# Copyright 2020 Open Source Robotics Foundation, Inc.
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

import time

from launch import LaunchService
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    spawn_targets = [

        Node(
            package='gazebo_ros',
            node_executable='spawn_entity.py',
            arguments=[
                '-database', 'MiR100', 
                '-reference_frame', 'mir100_0_placeholder',
                '-entity', 'mir100_0',
                '-z', '0.05']),

        Node(
            package='gazebo_ros',
            node_executable='spawn_entity.py',
            arguments=[
                '-database', 'MiR100', 
                '-reference_frame', 'mir100_1_placeholder',
                '-entity', 'mir100_1',
                '-z', '0.05']),
        
        Node(
            package='gazebo_ros',
            node_executable='spawn_entity.py',
            arguments=[
                '-database', 'Magni',
                '-reference_frame', 'magni_0_placeholder',
                '-entity', 'magni_0',
                '-z', '0.05']),
        
        Node(
            package='gazebo_ros',
            node_executable='spawn_entity.py',
            arguments=[
                '-database', 'Magni',
                '-reference_frame', 'magni_1_placeholder',
                '-entity', 'magni_1',
                '-z', '0.05']),

        Node(
            package='gazebo_ros',
            node_executable='spawn_entity.py',
            arguments=[
                '-database', 'Magni',
                '-reference_frame', 'magni_2_placeholder',
                '-entity', 'magni_2',
                '-z', '0.05']),

        
        Node(
            package='gazebo_ros',
            node_executable='spawn_entity.py',
            arguments=[
                '-database', 'Magni',
                '-reference_frame', 'magni_3_placeholder',
                '-entity', 'magni_3',
                '-z', '0.05']),

    ]

    for st in spawn_targets:
        ld = LaunchDescription([st])
        ls = LaunchService()
        ls.include_launch_description(ld)
        ls.run()
        time.sleep(5)
        ls.shutdown()
        
    return LaunchDescription([])
