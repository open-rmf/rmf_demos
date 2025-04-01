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
import argparse
import sys

import rclpy

from rclpy.action import ActionClient
from rclpy.node import Node

from rmf_task_msgs.action import DynamicEvent


class DynamicEventActionClient(Node):

    def __init__(self, argv=sys.argv):
        super().__init__('simple_action_client')
        parser = argparse.ArgumentParser()
        parser.add_argument(
            '-F',
            '--fleet',
            required=True,
            type=str,
            help='Fleet name'
        )
        parser.add_argument(
            '-R',
            '--robot',
            required=True,
            type=str,
            help='Robot name'
        )
        parser.add_argument(
            '-p',
            '--place',
            required=True,
            type=str,
            help='Place to go to',
        )

        self.args = parser.parse_args(argv[1:])

        topic = f'/rmf/dynamic_event/command/{self.args.fleet}/{self.args.robot}'
        self._action_client = ActionClient(self, DynamicEvent, topic)

        self.send_goal(self.args.place)

    def send_goal(self, order):
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()

        goal_msg = DynamicEvent.Goal()
        goal_msg.event_type = 1
        goal_msg.category = 'go_to_place'
        goal_msg.description = f'{{"waypoint": "{order}"}}'
        goal_msg.id = 2
        goal_msg.dynamic_event_seq = 1

        self.get_logger().info(f'Sending goal: {order}')
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            rclpy.shutdown()
            return

        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'[id: "{feedback.id}"] {feedback.status}')

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'[id: "{result.id}"] {result.status}')
        rclpy.shutdown()


def main(argv=sys.argv):
    """Go to place."""
    rclpy.init(args=sys.argv)
    args_without_ros = rclpy.utilities.remove_ros_args(sys.argv)

    task_requester = DynamicEventActionClient(args_without_ros)

    rclpy.spin(task_requester)


if __name__ == '__main__':
    main(sys.argv)
