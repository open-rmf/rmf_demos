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
import json
import sys

import rclpy

from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy as Durability
from rclpy.qos import QoSHistoryPolicy as History
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy as Reliability

from rmf_task_msgs.action import DynamicEvent
from rmf_task_msgs.msg import DynamicEventStatus


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
            type=str,
            nargs='+',
            help=(
                'Place(s) for the robot to go. If more than one is listed, '
                'the fleet adapter will choose the closest reachable place. '
                'This cannot be used with --file, --end, or --cancel.',
            ),
        )
        parser.add_argument(
            '-f',
            '--file',
            type=str,
            help=(
                'File containing a json object containing at least '
                '{{ "category": -, "description": - }}. This will be issued '
                'as the event request. This cannot be used with --place, '
                '--end, or --cancel.'
            ),
        )
        parser.add_argument(
            '-e',
            '--end',
            action=argparse.BooleanOptionalAction,
            help=(
                'Tell the dynamic event that it has ended. The task of the '
                'robot will proceed to its next phase. This cannot be used '
                'with --place, --file, or --cancel.'
            ),
        )
        parser.add_argument(
            '-c',
            '--cancel',
            action=argparse.BooleanOptionalAction,
            help=(
                'Cancel the currently executing child request of the dynamic '
                'event. After cancelling the child request, the dynamic event '
                'will wait for a new request. This cannot be used with '
                '--place, --file, or --end.'
            ),
        )
        parser.add_argument(
            '--stubbornness',
            type=float,
            default=0.0,
            help=(
                'How long (in seconds) the robot should report that it will '
                'stubbornly wait where it is for the next command to arrive. '
                'This only has an effect if used with --place or --file.'
            )
        )

        self.args = parser.parse_args(argv[1:])

        actions = {
            '--file': self.args.file,
            '--place': self.args.place,
            '--end': self.args.end,
            '--cancel': self.args.cancel,
        }
        chosen_actions = []
        for key, value in actions.items():
            if value:
                chosen_actions.append(key)

        if len(chosen_actions) > 1:
            raise RuntimeError(
                f'You have used multiple request types: {chosen_actions}. '
                f'You must choose exactly one of these.'
            )

        if not chosen_actions:
            raise RuntimeError(
                'You must use exactly one request type: '
                '--file, --place, --end, or --cancel'
            )

        self.request_file_contents = None
        if self.args.file:
            with open(self.args.file) as f:
                self.request_file_contents = json.load(f)

            if (
                'category' not in self.request_file_contents
                or 'description' not in self.request_file_contents
            ):
                raise RuntimeError(
                    'The input json file must have an object that contains '
                    'both a "category" and a "description" field.'
                )

        action_topic = (
            f'rmf/dynamic_event/command/{self.args.fleet}/{self.args.robot}'
        )
        self._action_client = ActionClient(self, DynamicEvent, action_topic)

        status_topic = (
            f'rmf/dynamic_event/status/{self.args.fleet}/{self.args.robot}'
        )

        self.received_status = False

        def on_status_update(status):
            if self.received_status:
                # We only need the status once
                return

            self.received_status = True
            print(f'Got status: {status}')
            self.status = status
            self.send_goal()

        transient_qos = QoSProfile(
            history=History.KEEP_LAST,
            depth=1,
            reliability=Reliability.RELIABLE,
            durability=Durability.TRANSIENT_LOCAL
        )

        self._status_subscription = self.create_subscription(
            DynamicEventStatus, status_topic, on_status_update, transient_qos,
        )
        print(
            f'Waiting for a dynamic event status update from '
            f'{self.args.fleet}/{self.args.robot} ...'
        )

    def send_goal(self):
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()

        goal_msg = DynamicEvent.Goal()

        if self.args.place:
            goal_msg.event_type = DynamicEvent.Goal.EVENT_TYPE_NEXT
            goal_msg.id = self.status.id + 1
            goal_msg.category = 'go_to_place'
            goal_msg.description = json.dumps({'one_of': self.args.place})

            description = {'one_of': self.args.place}
            goal_msg.description = json.dumps(description)

        if self.request_file_contents:
            goal_msg.event_type = DynamicEvent.Goal.EVENT_TYPE_NEXT
            goal_msg.id = self.status.id + 1
            goal_msg.category = self.request_file_contents['category']
            goal_msg.description = json.dumps(
                self.request_file_contents['description']
            )

        if self.args.end:
            goal_msg.event_type = DynamicEvent.Goal.EVENT_TYPE_FINISHED

        if self.args.cancel:
            goal_msg.event_type = DynamicEvent.Goal.EVENT_TYPE_CANCEL
            goal_msg.id = self.status.id

        goal_msg.dynamic_event_seq = self.status.dynamic_event_seq
        goal_msg.stubborn_period = self.args.stubbornness

        self.get_logger().info(f'Sending goal: {goal_msg}')
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
