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
import threading

import argparse
import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default

from rmf_door_msgs.msg import DoorState, DoorRequest

from fastapi import FastAPI
import uvicorn
from typing import Optional
from pydantic import BaseModel

app = FastAPI()


class Request(BaseModel):
    requested_mode: int


class Response(BaseModel):
    data: Optional[dict] = None
    success: bool
    msg: str


'''
    The DoorManager class simulates a bridge between the door API, that
    depends on the vendor and could be for example REST based, and the
    simulated doors that operate using ROS2 messages.
    Users can use this door to validate their door adapter in simulation
'''


class DoorManager(Node):

    def __init__(self, doors, namespace='sim'):
        super().__init__('door_manager')

        self.door_states = {}
        for door in doors:
            self.door_states[door] = None

        # Setup publisher and subscriber
        self.door_request_pub = self.create_publisher(
            DoorRequest,
            namespace + '/door_requests',
            qos_profile=qos_profile_system_default)

        self.door_state_sub = self.create_subscription(
            DoorState,
            namespace + '/door_states',
            self.door_state_cb,
            qos_profile=qos_profile_system_default)

        @app.get('/open-rmf/demo-door/door_state',
                 response_model=Response)
        async def state(door_name: str):
            response = {
                'data': {},
                'success': False,
                'msg': ''
            }

            if door_name not in self.door_states:
                self.get_logger().warn('Door not being managed')
                return response

            state = self.door_states[door_name]
            if state is None:
                return response

            response['data']['current_mode'] = state.current_mode.value
            response['success'] = True
            return response

        @app.post('/open-rmf/demo-door/door_request',
                  response_model=Response)
        async def request(door_name: str, mode: Request):
            req = DoorRequest()
            response = {
                'data': {},
                'success': False,
                'msg': ''
            }

            if door_name not in self.door_states:
                self.get_logger().warn(f'Door {door_name} not being managed')
                return response

            now = self.get_clock().now()
            req.door_name = door_name
            req.request_time = now.to_msg()
            req.requested_mode.value = mode.requested_mode
            req.requester_id = 'rmf_demos_door_manager'

            self.door_request_pub.publish(req)
            response['success'] = True
            return response

    def door_state_cb(self, msg):
        if msg.door_name not in self.door_states:
            return
        self.door_states[msg.door_name] = msg


def main(argv=sys.argv):
    args_without_ros = rclpy.utilities.remove_ros_args(argv)
    parser = argparse.ArgumentParser(
        prog='door_manager',
        description='Demo door manager')
    parser.add_argument('-c', '--config', required=True, type=str)
    args = parser.parse_args(args_without_ros[1:])

    with open(args.config, 'r') as f:
        config = yaml.safe_load(f)

    rclpy.init()
    node = DoorManager(config['doors'])

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,))
    spin_thread.start()

    uvicorn.run(app,
                host=config['door_manager']['ip'],
                port=config['door_manager']['port'],
                log_level='warning')

    rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv)
