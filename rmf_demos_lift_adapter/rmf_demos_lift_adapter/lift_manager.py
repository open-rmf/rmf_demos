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

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default

from rmf_lift_msgs.msg import LiftState, LiftRequest

from fastapi import FastAPI
import uvicorn
from typing import Optional
from pydantic import BaseModel

app = FastAPI()


class Request(BaseModel):
    floor: str
    door_state: int


class Response(BaseModel):
    data: Optional[dict] = None
    success: bool
    msg: str


'''
    The LiftManager class simulates a bridge between the lift API, that
    depends on the vendor and could be for example REST based, and the
    simulated lifts that operate using ROS2 messages.
    Users can use this lift to validate their lift adapter in simulation
'''


class LiftManager(Node):

    def __init__(self, namespace='sim'):
        super().__init__('lift_manager')

        self.address = self.declare_parameter('manager_address', 'localhost').value
        self.port = self.declare_parameter('manager_port', 5003).value

        self.lift_states = {}

        # Setup publisher and subscriber
        self.lift_request_pub = self.create_publisher(
            LiftRequest,
            namespace + '/lift_requests',
            qos_profile=qos_profile_system_default)

        self.lift_state_sub = self.create_subscription(
            LiftState,
            namespace + '/lift_states',
            self.lift_state_cb,
            qos_profile=qos_profile_system_default)

        @app.get('/open-rmf/demo-lift/lift_state',
                 response_model=Response)
        async def state(lift_name: str):
            response = {
                'data': {},
                'success': False,
                'msg': ''
            }

            if lift_name not in self.lift_states:
                self.get_logger().warn(f'Lift {lift_name} not being managed')
                return response

            state = self.lift_states[lift_name]

            response['data']['available_floors'] = state.available_floors
            response['data']['current_floor'] = state.current_floor
            response['data']['destination_floor'] = state.destination_floor
            response['data']['door_state'] = state.door_state
            response['data']['motion_state'] = state.motion_state
            response['success'] = True
            return response

        @app.get('/open-rmf/demo-lift/lift_names',
                 response_model=Response)
        async def lift_names():
            response = {
                'data': {},
                'success': False,
                'msg': ''
            }

            response['data']['lift_names'] = [name for name in self.lift_states]
            response['success'] = True
            return response

        @app.post('/open-rmf/demo-lift/lift_request',
                  response_model=Response)
        async def request(lift_name: str, floor: Request):
            req = LiftRequest()
            response = {
                'data': {},
                'success': False,
                'msg': ''
            }

            if lift_name not in self.lift_states:
                self.get_logger().warn(f'Lift {lift_name} not being managed')
                return response

            now = self.get_clock().now()
            req.lift_name = lift_name
            req.request_time = now.to_msg()
            req.request_type = req.REQUEST_AGV_MODE
            req.door_state = floor.door_state
            req.destination_floor = floor.floor
            req.session_id = 'rmf_demos_lift_adapter'

            self.lift_request_pub.publish(req)
            response['success'] = True
            return response

    def lift_state_cb(self, msg):
        self.lift_states[msg.lift_name] = msg


def main(argv=sys.argv):
    rclpy.init()
    node = LiftManager()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,))
    spin_thread.start()

    uvicorn.run(app,
                host=node.address,
                port=node.port,
                log_level='warning')

    rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv)
