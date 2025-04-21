#!/usr/bin/env python3

# Copyright 2025 Open Source Robotics Foundation, Inc.
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
"""Wait for a task to be completed."""

import argparse
import asyncio
import json
import sys
import time
import uuid

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy as Durability
from rclpy.qos import QoSHistoryPolicy as History
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy as Reliability

from rmf_fleet_msgs.msg import FleetState
from rmf_task_msgs.msg import ApiRequest


class TaskObserver(Node):
    """
    This is a tool that should be used only for testing purpose.

    Do not use it in production!
    """

    def __init__(self, parser):
        """Initialize the TaskObserver."""
        super().__init__('TaskObserver')

        self.parser = parser
        self.response = asyncio.Future()

        self.subscription = self.create_subscription(
            FleetState,
            '/fleet_states',
            self.state_watcher,
            10)

        transient_qos = QoSProfile(
            history=History.KEEP_LAST,
            depth=1,
            reliability=Reliability.RELIABLE,
            durability=Durability.TRANSIENT_LOCAL,
        )
        self.pub = self.create_publisher(
            ApiRequest, 'task_api_requests', transient_qos
        )

    def state_watcher(self, fleet_state: FleetState):
        """Watch the fleet state."""
        if fleet_state.name != self.parser.fleet:
            return
        for robot_state in fleet_state.robots:
            if robot_state.name == self.parser.robot:
                if robot_state.task_id == '':
                    print('Robot not performing a task')
                    self.response.set_result(robot_state)
                    return
                else:
                    msg = ApiRequest()
                    msg.request_id = 'cancel_task_' + str(uuid.uuid4())
                    payload = {}
                    payload['type'] = 'cancel_task_request'
                    payload['task_id'] = robot_state.task_id

                    msg.json_msg = json.dumps(payload)
                    print(f'msg: \n{json.dumps(payload, indent=2)}')
                    self.pub.publish(msg)
                    self.response.set_result(robot_state)
                    return


def create_parser():
    """Create the parser for the script."""
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
    return parser


def main(argv=sys.argv):
    """Wait for a task to be complete."""
    rclpy.init(args=sys.argv)
    args_without_ros = rclpy.utilities.remove_ros_args(sys.argv)
    arg_parser = create_parser()
    arguments = arg_parser.parse_args(args_without_ros[1:])
    task_requester = TaskObserver(arguments)
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
