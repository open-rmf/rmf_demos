#!/usr/bin/env python3

# Copyright 2024 Open Source Robotics Foundation, Inc.
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
"""Get the location of a robot."""

import argparse
import asyncio
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy as Durability
from rclpy.qos import QoSHistoryPolicy as History
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy as Reliability

from rmf_building_map_msgs.msg import Graph

from rmf_fleet_msgs.msg import FleetState


class RobotStateObserver(Node):
    """
    This is a tool that should be used only for testing purpose.

    Do not use it in production!
    """

    def __init__(self, parser):
        """Initialize the observer."""
        super().__init__('TaskObserver')

        self.parser = parser
        self.response = asyncio.Future()

        self.subscription = self.create_subscription(
            FleetState,
            '/fleet_states',
            self.state_watcher,
            10)

        nav_graph_qos = QoSProfile(
            history=History.KEEP_LAST,
            depth=10,
            durability=Durability.TRANSIENT_LOCAL,
            reliability=Reliability.RELIABLE)

        self.nav_graph_subscription = self.create_subscription(
            Graph,
            '/nav_graphs',
            self.nav_graph_watcher,
            nav_graph_qos)

        self.nav_graph = None

    def state_watcher(self, fleet_state: FleetState):
        """Watch the fleet state."""
        if self.nav_graph is None:
            print('Nav graph not found')
            return

        if fleet_state.name != self.parser.fleet:
            return
        for robot_state in fleet_state.robots:
            if robot_state.name == self.parser.robot:
                for graph_node in self.nav_graph.vertices:
                    dist = (graph_node.x - robot_state.location.x) ** 2
                    dist += (graph_node.y - robot_state.location.y) ** 2
                    name = graph_node.name
                    if dist < 0.5:
                        if not self.parser.block_until_reaches:
                            self.response.set_result(name)
                        elif self.parser.block_until_reaches == name:
                            self.response.set_result(name)
                        return

    def nav_graph_watcher(self, navgraph: Graph):
        """Watch the nav graph."""
        if navgraph.name == self.parser.fleet:
            self.nav_graph = navgraph


def create_parser():
    """Create the parser."""
    parser = argparse.ArgumentParser()

    parser.add_argument(
        '-R',
        '--robot',
        type=str,
        help='Robot name, should define together with fleet',
    )

    parser.add_argument(
        '-F',
        '--fleet',
        type=str,
        help='fleet name',
    )

    parser.add_argument(
        '--timeout',
        type=int,
        help='Timeout seconds',
    )

    parser.add_argument(
        '--block-until-reaches',
        '-B',
        type=str,
        help='Block until this waypoint is reached. If empty,' +
        ' then the command does not block.'
    )
    return parser


def main(argv=sys.argv):
    """Get a robot location."""
    rclpy.init(args=sys.argv)
    args_without_ros = rclpy.utilities.remove_ros_args(sys.argv)
    arg_parser = create_parser()
    arguments = arg_parser.parse_args(args_without_ros[1:])
    task_requester = RobotStateObserver(arguments)
    start_time = time.time()
    rclpy.spin_until_future_complete(
        task_requester, task_requester.response, timeout_sec=arguments.timeout
    )
    if task_requester.response.done():
        print(f'Got response: \n{task_requester.response.result()}')
        end_time = time.time()
        elapsed = end_time - start_time
        print(f'elapsed time: {elapsed}')
    else:
        print('Timed out')
        sys.exit(-1)
    rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv)
