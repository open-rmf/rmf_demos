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
import math
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
        # squared tolerance (m^2): radius of the search window around the robot
        self.tolerance_squared = (self.parser.tolerance) ** 2
        asyncio.set_event_loop(asyncio.new_event_loop())
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
                best_waypoint = None
                # running minimum squared distance (m^2); a vertex wins only if
                # it is within the inclusive tolerance radius AND strictly
                # closer than the best vertex seen so far
                best_dist_squared = math.inf
                for graph_node in self.nav_graph.vertices:
                    node_floor = ''
                    for p in graph_node.params:
                        if p.name == 'map_name':
                            node_floor = p.value_string
                            break

                    if (node_floor and
                            node_floor != robot_state.location.level_name):
                        continue

                    dist_sq = (graph_node.x - robot_state.location.x) ** 2
                    dist_sq += (graph_node.y - robot_state.location.y) ** 2
                    if dist_sq <= self.tolerance_squared and dist_sq < best_dist_squared:
                        best_dist_squared = dist_sq
                        best_waypoint = graph_node.name

                if best_waypoint is None:
                    return

                if not self.parser.block_until_reaches:
                    self.response.set_result(best_waypoint)
                elif self.parser.block_until_reaches == best_waypoint:
                    self.response.set_result(best_waypoint)

    def nav_graph_watcher(self, navgraph: Graph):
        """Watch the nav graph."""
        if navgraph.name == self.parser.fleet:
            self.nav_graph = navgraph

            if self.parser.block_until_reaches:
                valid_waypoints = {node.name for node in navgraph.vertices}
                if self.parser.block_until_reaches not in valid_waypoints:
                    err_msg = (
                        f'Error: Waypoint [{self.parser.block_until_reaches}] '
                        'not found in nav graph for '
                        f'fleet [{self.parser.fleet}].')
                    print(err_msg)
                    if not self.response.done():
                        self.response.set_result(err_msg)


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

    parser.add_argument(
        '--tolerance',
        type=float,
        default=0.5,
        help='Search radius in meters for matching a vertex near the '
        'robot (squared internally as m^2) [default: 0.5]'
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
