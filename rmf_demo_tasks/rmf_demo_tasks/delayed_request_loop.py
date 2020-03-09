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

import sys
import argparse
from time import sleep

import rclpy
from rclpy.node import Node
from rmf_fleet_msgs import FleetState
from rmf_task_msgs.msg import Loop



class DelayedLoopRequester(Node):

    def __init__(self, argv=sys.argv):
        super().__init__('delayed_loop_requester_node')

        parser = argparse.ArgumentParser()
        parser.add_argument('-s', '--start', help='Start waypoint')
        parser.add_argument('-f', '--finish', help='Finish waypoint')
        parser.add_argument('-n', '--num', help='Number of loops to perform', 
                type=int, default=1)
        parser.add_argument('-i', '--task-id', help='Task ID', default='', 
                type=str)
        parser.add_argument('-r', '--robot-type', help='Type of robot', 
                default='magni')
        parser.add_argument('--delay',
                help='Number of secs to wait before sending out the request',
                default=3, type=int)
        parser.add_argument('--timout', 
                help='Number of secs before this node timesout',
                default=10, type=int)
        parser.add_argument('--robot-name', 
                help='Robot name to wait for in fleet state message', 
                default='', type=str)
        args = parser.parse_args(argv[1:])

        self.publisher = self.create_publisher(Loop, 'loop_requests', 10)
        self.subscription = self.create_subscription(FleetState, 
                'fleet_states', self.callback, 10)
    

    def callback(self, msg):
        raise NotImplementedError


if __name__ == '__main__':
    rclpy.init(args=sys.argv)
    args_without_ros = rclpy.utilities.remove_ros_args(sys.argv)

    delayed_loop_requester = DelayedLoopRequester(args_without_ros)
    delayed_loop_requester.main()
    rclpy.shutdown()
