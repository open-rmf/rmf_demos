#!/usr/bin/env python3

# Copyright 2026 Open Source Robotics Foundation, Inc.
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
"""Dispatch a zone task."""

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


def parse_hint(text):
    """Turn one -H argument into the hint object the zone schema takes."""
    parts = text.split(':')
    kind = parts[0]
    name = ''
    if len(parts) > 1:
        name = parts[1]
    hint = {}
    if kind == 'wp':
        hint['waypoint'] = name
        rest = parts[2:]
    elif kind == 'group':
        hint['group'] = name
        rest = parts[2:]
    elif kind == 'any':
        rest = parts[1:]
    else:
        raise argparse.ArgumentTypeError(f'unknown hint type [{kind}]')
    if kind != 'any' and not name:
        raise argparse.ArgumentTypeError(f'{kind} needs a name')
    if len(rest) > 1:
        raise argparse.ArgumentTypeError(
            'orientations go in one comma separated list')
    if rest and rest[0]:
        hint['orientations'] = []
        for angle in rest[0].split(','):
            hint['orientations'].append(float(angle))
    return hint


class TaskRequester(Node):
    """Dispatch zone task requester."""

    def __init__(self, argv=sys.argv):
        """Initialize task requester."""
        super().__init__('task_requester')
        parser = argparse.ArgumentParser()
        parser.add_argument(
            '-z',
            '--zone',
            required=True,
            type=str,
            help='Name of the target zone',
        )
        parser.add_argument(
            '-H',
            '--hint',
            action='append',
            type=parse_hint,
            default=None,
            metavar='TYPE[:NAME][:ANGLE,...]',
            help='Add a preference, most preferred first. TYPE is wp, '
                 'group or any, and any takes no name. Orientations are '
                 'radians. Examples: wp:wait_1:1.57,4.71  group:C:0.7  '
                 'any:1.57',
        )
        parser.add_argument(
            '-F',
            '--fleet',
            type=str,
            help='Fleet name, should define together with robot',
        )
        parser.add_argument(
            '-R',
            '--robot',
            type=str,
            help='Robot name, should define together with fleet',
        )
        parser.add_argument(
            '-st',
            '--start_time',
            help='Start time from now in secs, default: 0',
            type=int,
            default=0,
        )
        parser.add_argument(
            '-pt',
            '--priority',
            help='Priority value for this request (0 or 1)',
            type=int,
            default=None,
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

        self.args = parser.parse_args(argv[1:])
        asyncio.set_event_loop(asyncio.new_event_loop())
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
        msg.request_id = 'zone_' + str(uuid.uuid4())
        payload = {}

        if self.args.robot and self.args.fleet:
            self.get_logger().info("Using 'robot_task_request'")
            payload['type'] = 'robot_task_request'
            payload['robot'] = self.args.robot
            payload['fleet'] = self.args.fleet
        else:
            self.get_logger().info("Using 'dispatch_task_request'")
            payload['type'] = 'dispatch_task_request'

        request = {}

        # Set task request request time and start time
        now = self.get_clock().now().to_msg()
        now.sec = now.sec + self.args.start_time
        start_time = now.sec * 1000 + round(now.nanosec / 10**6)
        request['unix_millis_request_time'] = start_time
        request['unix_millis_earliest_start_time'] = start_time
        if self.args.priority is not None:
            request['priority'] = {
                'type': 'binary',
                'value': 1 if self.args.priority > 0 else 0,
            }

        request['requester'] = self.args.requester

        # Define task request category
        request['category'] = 'zone'

        if self.args.fleet:
            request['fleet_name'] = self.args.fleet

        description = {'zone_name': self.args.zone}

        if self.args.hint:
            description['hints'] = self.args.hint

        request['description'] = description
        payload['request'] = request
        msg.json_msg = json.dumps(payload)

        def receive_response(response_msg: ApiResponse):
            if response_msg.request_id == msg.request_id:
                self.response.set_result(json.loads(response_msg.json_msg))

        transient_qos.depth = 10
        self.sub = self.create_subscription(
            ApiResponse, 'task_api_responses', receive_response, transient_qos
        )

        print(f'Json msg payload: \n{json.dumps(payload, indent=2)}')
        self.pub.publish(msg)


###############################################################################


def main(argv=sys.argv):
    """Dispatch a zone task."""
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
