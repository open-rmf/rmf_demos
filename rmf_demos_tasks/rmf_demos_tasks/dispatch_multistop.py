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

import sys
import uuid
import argparse
import json
import asyncio

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_system_default
from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy as History
from rclpy.qos import QoSDurabilityPolicy as Durability
from rclpy.qos import QoSReliabilityPolicy as Reliability

from rmf_task_msgs.msg import ApiRequest, ApiResponse

def list_of_labels(arg):
    final_kvs = []
    temp_kvs = []
    for p in arg.split(","):
        if "=" in p:
            final_kvs.append(",".join(temp_kvs))
            temp_kvs.clear()
            temp_kvs.append(p)
        else:
            temp_kvs.append(p)
    final_kvs.append(",".join(temp_kvs))
    temp_kvs.clear()
    return final_kvs[1:]


###############################################################################

class TaskRequester(Node):

    def __init__(self, argv=sys.argv):
        super().__init__('task_requester')
        parser = argparse.ArgumentParser()
        parser.add_argument('-F', '--fleet', required=False, default='',
                            type=str, help='Fleet name')
        parser.add_argument('-R', '--robot', required=False, default='',
                            type=str, help='Robot name')
        parser.add_argument('-g', '--go_to', required=True, nargs='+',
                            type=str, help='Places to go to for delivery')
        parser.add_argument('-e', '--end_collection', required=False,
                            default='', type=str, help='End collection')
        parser.add_argument('-t', '--timeout', required=False, default=30,
                            type=int, help='Number of seconds to timeout')
        parser.add_argument('-st', '--start_time',
                            help='Start time from now in secs, default: 0',
                            type=int, default=0)
        parser.add_argument('-pt', '--priority',
                            help='Priority value for this request',
                            type=int, default=0)
        parser.add_argument('-L', '--labels', required=False, default=[],
                            type=list_of_labels, help='labels to pass to the web server')
        parser.add_argument("--use_sim_time", action="store_true",
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
            self.get_logger().info("Using Sim Time")
            param = Parameter("use_sim_time", Parameter.Type.BOOL, True)
            self.set_parameters([param])

        # Construct task
        msg = ApiRequest()
        msg.request_id = f"multistop_" + str(uuid.uuid4())
        payload = {}
        if self.args.fleet and self.args.robot:
            payload["type"] = "robot_task_request"
            payload["robot"] = self.args.robot
            payload["fleet"] = self.args.fleet
        else:
            payload["type"] = "dispatch_task_request"
        request = {}

        # Set task request start time
        now = self.get_clock().now().to_msg()
        now.sec = now.sec + self.args.start_time
        start_time = now.sec * 1000 + round(now.nanosec/10**6)
        request["unix_millis_earliest_start_time"] = start_time
        # todo(YV): Fill priority after schema is added

        # Define task request category
        request["category"] = "compose"

        # Define task request description with phases
        description = {}  # task_description_Compose.json
        description["category"] = "multistop"
        description["phases"] = []

        # Multistop activity (GoToPlace + wait)
        for i in range(len(self.args.go_to)):
            place = self.args.go_to[i]
            # Add GoToPlace activity
            go_to_place_activity = [{
                "category": "go_to_place",
                "description": place
            }]
            description["phases"].append({
                "activity": {
                    "category": "sequence",
                    "description": {"activities": go_to_place_activity
            }}})
            # Add wait activity
            if i == len(self.args.go_to) - 1:
                wait_activity = [{
                    "category": "perform_action",
                    "description": {
                        "unix_millis_action_duration_estimate": 60000,
                        "category": 'wait_until',
                        "description": {
                            "timeout": self.args.timeout,
                            "location": place,
                            "compartment": "top_or_bottom",
                            "delivery_action": "pickup_or_dropoff",
                            "last_location": True
                }}}]
            else:
                wait_activity = [{
                    "category": "perform_action",
                    "description": {
                        "unix_millis_action_duration_estimate": 60000,
                        "category": 'wait_until',
                        "description": {
                            "timeout": self.args.timeout,
                            "location": place,
                            "compartment": "top_or_bottom",
                            "delivery_action": "pickup_or_dropoff"
                }}}]
            description["phases"].append({
                "activity": {
                    "category": "sequence",
                    "description": {
                        "activities": wait_activity
            }}})

        # End collection activity (GoToPlace + wait)
        if self.args.end_collection != '':
            # Add GoToPlace activity
            go_to_place_activity = [{
                "category": "go_to_place",
                "description": self.args.end_collection}]
            description["phases"].append({
                "activity": {
                "category": "sequence",
                "description": {"activities": go_to_place_activity}}})
            # Add wait activity
            wait_activity = [{
                "category": "perform_action",
                "description": {
                    "unix_millis_action_duration_estimate": 60000,
                    "category": 'wait_until',
                    "description": {
                        "timeout": self.args.timeout,
                        "location": self.args.end_collection,
                        "compartment": "top_or_bottom",
                        "delivery_action": "pickup_or_dropoff",
                        "end_collection": True
            }}}]
            description["phases"].append({
                "activity": {
                    "category": "sequence",
                    "description": {"activities": wait_activity}}})

        # Consolidate
        request["description"] = description
        if self.args.labels != []:
            request["labels"] = self.args.labels
        payload["request"] = request
        msg.json_msg = json.dumps(payload)

        def receive_response(response_msg: ApiResponse):
            if response_msg.request_id == msg.request_id:
                self.response.set_result(json.loads(response_msg.json_msg))

        transient_qos.depth = 10
        self.sub = self.create_subscription(
            ApiResponse, 'task_api_responses', receive_response, transient_qos
        )

        print(f"Json msg payload: \n{json.dumps(payload, indent=2)}")
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
