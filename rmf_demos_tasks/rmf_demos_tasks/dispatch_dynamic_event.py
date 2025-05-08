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
"""Dynamic Event."""

import argparse
import asyncio
import json
import sys
import uuid

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSDurabilityPolicy as Durability
from rclpy.qos import QoSHistoryPolicy as History
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy as Reliability

from rmf_task_msgs.msg import ApiRequest
from rmf_task_msgs.msg import ApiResponse

###############################################################################


class TaskRequester(Node):
    """Task requester."""

    def __init__(self, argv=sys.argv):
        """Initialize a task requester."""
        super().__init__('task_requester')
        parser = argparse.ArgumentParser()
        parser.add_argument('-F', '--fleet', type=str, help='Fleet name')
        parser.add_argument('-R', '--robot', type=str, help='Robot name')
        parser.add_argument(
            '-p',
            '--place',
            type=str,
            nargs='+',
            help=(
                'Place(s) for the fleet adapter to estimate the robot will '
                'go. If more than one is listed, the fleet adapter will '
                'choose the closest reachable place. This cannot be used with '
                '--file.',
            ),
        )
        parser.add_argument(
            '-f',
            '--file',
            type=str,
            help=(
                'File containing a json object containing at least '
                '{{ "category": -, "description": - }}. The contents of this '
                'file will be used as the estimate of the dynamic event. This '
                'cannot be used with --place.'
            ),
        )
        parser.add_argument(
            '--use_sim_time',
            action='store_true',
            help='Use sim time, default: false',
        )
        parser.add_argument(
            '--requester',
            help='Entity that is requesting this task',
            type=str,
            default='rmf_demos_tasks'
        )
        parser.add_argument(
            '-r',
            '--required',
            type=str,
            nargs='+',
            help=(
                'Add an action capability requirement for the selected fleet. '
                'This takes the form of a file containing '
                '{{ "category": -, "description": - }}, similar to --file. '
                'The fleet must be able to execute the described event. More '
                'than one file can be specified and each one will be required.'
            ),
        )
        parser.add_argument(
            '--parameters',
            type=str,
            help=(
                'File containing a json object for the parameters of the '
                'dynamic event. These parameters will be published with the '
                'dynamic event information when the event begins.'
            ),
        )
        parser.add_argument(
            '--detail',
            type=str,
            help='Human-friendly detailed description of the dynamic event.',
        )

        self.args = parser.parse_args(argv[1:])

        if self.args.file and self.args.place:
            raise RuntimeError(
                'You cannot use both --file and --place at the same time.'
            )

        request_file_contents = None
        if self.args.file:
            with open(self.args.file) as f:
                request_file_contents = json.load(f)

            if (
                'category' not in request_file_contents
                or 'description' not in request_file_contents
            ):
                raise RuntimeError(
                    f'The input json file {self.args.file} must have an '
                    f'object that contains both a "category" and a '
                    '"description" field.'
                )

        event_parameters = None
        if self.args.parameters:
            with open(self.args.parameters) as f:
                event_parameters = json.load(f)

        required = []
        if self.args.required:
            for req_file in self.args.required:
                with open(req_file) as f:
                    req_contents = json.load(f)

                if (
                    'category' not in req_contents
                    or 'description' not in req_contents
                ):
                    raise RuntimeError(
                        f'The input json file {req_file} must have an object '
                        'that contains both a "category" and a "description" '
                        'field.'
                    )

                required.append(req_contents)

        self.response = asyncio.Future()

        transient_qos = QoSProfile(
            history=History.KEEP_LAST,
            depth=1,
            reliability=Reliability.RELIABLE,
            durability=Durability.TRANSIENT_LOCAL,
        )

        self.pub = self.create_publisher(
            ApiRequest, 'task_api_requests', transient_qos
        )

        # enable ros sim time
        if self.args.use_sim_time:
            self.get_logger().info('Using Sim Time')
            param = Parameter('use_sim_time', Parameter.Type.BOOL, True)
            self.set_parameters([param])

        # Construct task
        msg = ApiRequest()
        msg.request_id = 'direct_' + str(uuid.uuid4())
        payload = {}

        if self.args.robot and self.args.fleet:
            self.get_logger().info("Using 'robot_task_request'")
            payload['type'] = 'robot_task_request'
            payload['robot'] = self.args.robot
            payload['fleet'] = self.args.fleet
        else:
            self.get_logger().info("Using 'dispatch_task_request'")
            payload['type'] = 'dispatch_task_request'

        # Set task request start time
        now = self.get_clock().now().to_msg()
        start_time = now.sec * 1000 + round(now.nanosec / 10**6)
        # todo(YV): Fill priority after schema is added

        category = None
        estimate = None
        if self.args.place:
            category = 'go_to_place'
            estimate = {
                'category': 'go_to_place',
                'description': {
                    'one_of': self.args.place
                }
            }

        if request_file_contents:
            category = request_file_contents['category']
            estimate = request_file_contents

        description = {
            'required': required,
        }

        if category:
            description['category'] = category

        if estimate:
            description['estimate'] = estimate

        if event_parameters:
            description['parameters'] = event_parameters

        if self.args.detail:
            description['detail'] = self.args.detail

        dynamic_event_activity = {
            'category': 'dynamic_event',
            'description': description,
        }

        rmf_task_request = {
            'category': 'compose',
            'description': {
                'category': 'dynamic_event',
                'phases': [{'activity': dynamic_event_activity}],
            },
            'unix_millis_request_time': start_time,
            'unix_millis_earliest_start_time': start_time,
            'requester': self.args.requester,
        }

        if self.args.fleet:
            rmf_task_request['fleet_name'] = self.args.fleet

        payload['request'] = rmf_task_request

        msg.json_msg = json.dumps(payload)

        def receive_response(response_msg: ApiResponse):
            if response_msg.request_id == msg.request_id:
                self.response.set_result(json.loads(response_msg.json_msg))

        self.sub = self.create_subscription(
            ApiResponse, 'task_api_responses', receive_response, 10
        )

        print(f'Json msg payload: \n{json.dumps(payload, indent=2)}')

        self.pub.publish(msg)


###############################################################################


def main(argv=sys.argv):
    """Go to place."""
    rclpy.init(args=sys.argv)
    args_without_ros = rclpy.utilities.remove_ros_args(sys.argv)

    task_requester = TaskRequester(args_without_ros)
    rclpy.spin_until_future_complete(
        task_requester, task_requester.response, timeout_sec=5.0
    )
    if task_requester.response.done():
        print(f'Got response: \n{task_requester.response.result()}')
    else:
        print('Did not get a response')
    rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv)
