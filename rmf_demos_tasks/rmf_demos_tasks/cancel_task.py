#!/usr/bin/env python3

# Copyright 2022 Open Source Robotics Foundation, Inc.
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
import uuid

import rclpy
from rclpy.node import Node
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
            '-id',
            '--task_id',
            required=True,
            default='',
            type=str,
            help='Cancel Task ID',
        )

        self.args = parser.parse_args(argv[1:])

        transient_qos = QoSProfile(
            history=History.KEEP_LAST,
            depth=1,
            reliability=Reliability.RELIABLE,
            durability=Durability.TRANSIENT_LOCAL,
        )

        self.pub = self.create_publisher(
            ApiRequest, 'task_api_requests', transient_qos
        )

        self.response = rclpy.Future()

        def receive_response(response_msg: ApiResponse):
            if response_msg.request_id == msg.request_id:
                self.response.set_result(json.loads(response_msg.json_msg))

        self.sub = self.create_subscription(
            ApiResponse, 'task_api_responses', receive_response, transient_qos
        )

        # Construct task
        msg = ApiRequest()
        msg.request_id = 'cancel_task_' + str(uuid.uuid4())
        payload = {}
        payload['type'] = 'cancel_task_request'
        payload['task_id'] = self.args.task_id

        msg.json_msg = json.dumps(payload)
        print(f'msg: \n{json.dumps(payload, indent=2)}')
        self.pub.publish(msg)


###############################################################################


def main(argv=sys.argv):
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
