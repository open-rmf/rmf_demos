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

    def __init__(self, argv=sys.argv):
        super().__init__('task_requester')
        parser = argparse.ArgumentParser()
        parser.add_argument(
            '-c', '--category',
            type=str,
            default='compose',
            help='Set the category of the task'
        )
        parser.add_argument(
            '-f', '--file',
            required=True,
            type=str,
            help='File containing a task description formatted in json'
        )
        parser.add_argument(
            '-F', '--fleet',
            type=str,
            help='Fleet name, should define together with robot'
        )
        parser.add_argument(
            '-R', '--robot',
            type=str,
            help='Robot name, should define together with fleet'
        )
        parser.add_argument(
            '-st', '--start_time',
            type=int,
            default=0,
            help='Start time from now in secs, default: 0'
        )
        parser.add_argument(
            '-pt', '--priority',
            type=int,
            default=0,
            help='Priority value for this request'
        )
        parser.add_argument(
            '--use_sim_time',
            action='store_true',
            help='Use sim time, default: false'
        )

        self.args = parser.parse_args(argv[1:])
        self.response = asyncio.Future()

        with open(self.args.file) as f:
            description = json.load(f)

        transient_qos = QoSProfile(
            history=History.KEEP_LAST,
            depth=1,
            reliability=Reliability.RELIABLE,
            durability=Durability.TRANSIENT_LOCAL)
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
        msg.request_id = 'task_' + str(uuid.uuid4())
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

        # Set task request start time
        now = self.get_clock().now().to_msg()
        now.sec = now.sec + self.args.start_time
        start_time = now.sec * 1000 + round(now.nanosec/10**6)
        request['unix_millis_earliest_start_time'] = start_time
        # todo(YV): Fill priority after schema is added

        # Define task request category
        request['category'] = self.args.category

        # Define task request description
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
    rclpy.init(args=sys.argv)
    args_without_ros = rclpy.utilities.remove_ros_args(sys.argv)

    task_requester = TaskRequester(args_without_ros)
    rclpy.spin_until_future_complete(
        task_requester, task_requester.response, timeout_sec=5.0)
    if task_requester.response.done():
        print(f'Got response:\n{task_requester.response.result()}')
    else:
        print('Did not get a response')
    rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv)
