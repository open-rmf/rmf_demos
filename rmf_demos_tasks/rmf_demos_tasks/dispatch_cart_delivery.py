#!/usr/bin/env python3

# Copyright 2023 Open Source Robotics Foundation, Inc.
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
"""Dispatch a cart delivery."""

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

from rmf_task_msgs.msg import ApiRequest, ApiResponse


###############################################################################

class TaskRequester(Node):
    """Task requester."""

    def __init__(self, argv=sys.argv):
        """Initialize a task requester."""
        super().__init__('task_requester')
        parser = argparse.ArgumentParser()
        parser.add_argument('-p', '--pickup', required=True,
                            type=str,
                            help='Pickup waypoint name')
        parser.add_argument('-d', '--dropoff', required=True,
                            type=str,
                            help='Dropoff waypoint name')
        parser.add_argument('-F', '--fleet', type=str,
                            help='Fleet name, should define tgt with robot')
        parser.add_argument('-R', '--robot', type=str,
                            help='Robot name, should define tgt with fleet')
        parser.add_argument('-st', '--start_time',
                            help='Start time from now in secs, default: 0',
                            type=int, default=0)
        parser.add_argument('-pt', '--priority',
                            help='Priority value for this request',
                            type=int, default=0)
        parser.add_argument('--use_sim_time', action='store_true',
                            help='Use sim time, default: false')

        self.args = parser.parse_args(argv[1:])
        self.response = asyncio.Future()

        transient_qos = QoSProfile(
            history=History.KEEP_LAST,
            depth=1,
            reliability=Reliability.RELIABLE,
            durability=Durability.TRANSIENT_LOCAL)

        self.pub = self.create_publisher(
          ApiRequest, 'task_api_requests', transient_qos)

        # enable ros sim time
        if self.args.use_sim_time:
            self.get_logger().info('Using Sim Time')
            param = Parameter('use_sim_time', Parameter.Type.BOOL, True)
            self.set_parameters([param])

        # Construct task
        msg = ApiRequest()
        msg.request_id = 'delivery_' + str(uuid.uuid4())
        payload = {}
        if self.args.fleet and self.args.robot:
            self.get_logger().info("Using 'robot_task_request'")
            payload['type'] = 'robot_task_request'
            payload['robot'] = self.args.robot
            payload['fleet'] = self.args.fleet
        else:
            self.get_logger().info("Using 'dispatch_task_request'")
            payload['type'] = 'dispatch_task_request'
        request = {}

        # Set task request start time
        now = self.get_clock().now().to_msg()
        now.sec = now.sec + self.args.start_time
        start_time = now.sec * 1000 + round(now.nanosec/10**6)
        request['unix_millis_earliest_start_time'] = start_time

        # TODO(luca) expose duration and tool sink to CLI
        def __create_perform_action(action_category, duration_ms=10000,
                                    use_tool_sink=False):
            return {
                    'unix_millis_action_duration_estimate': duration_ms,
                    # for internal FleetUpdateHandle to check if action
                    # is performable by this fleet
                    'category': action_category,
                    'description':
                    {
                        # for fleet manager to start action process
                        'deliver_cart_task_name': action_category
                    },
                    'use_tool_sink': use_tool_sink
                    }

        # Define with request category compose
        request['category'] = 'compose'

        # Define task request description with phases
        description = {}  # task_description_Compose.json
        description['category'] = 'deliver_cart'
        description['phases'] = []
        activities = []
        # Add each phase
        activities.append({
            'category': 'go_to_place',
            'description': self.args.pickup})
        activities.append({
            'category': 'perform_action',
            'description': __create_perform_action('delivery_pickup')})
        activities.append({
            'category': 'go_to_place',
            'description': self.args.dropoff})
        activities.append({
            'category': 'perform_action',
            'description': __create_perform_action('delivery_dropoff')})
        # Add activities to phases
        description['phases'].append(
            {'activity': {
                'category': 'sequence',
                'description': {'activities': activities}}})

        request['description'] = description
        payload['request'] = request
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
    """Dispatch a cart delivery."""
    rclpy.init(args=sys.argv)
    args_without_ros = rclpy.utilities.remove_ros_args(sys.argv)

    task_requester = TaskRequester(args_without_ros)
    rclpy.spin_until_future_complete(
        task_requester, task_requester.response, timeout_sec=5.0)
    if task_requester.response.done():
        print(f'Got response: \n{task_requester.response.result()}')
    else:
        print('Did not get a response')
    rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv)
