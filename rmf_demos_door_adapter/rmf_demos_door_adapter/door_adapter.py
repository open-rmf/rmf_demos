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
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from rmf_door_msgs.msg import DoorMode, DoorState, DoorRequest

from .DoorAPI import DoorAPI

'''
    The DemoDoorAdapter is a node which provide updates to Open-RMF, as well
    as handle incoming requests to control the integrated door, by calling the
    implemented functions in DoorAPI.
'''


class DemoDoorAdapter(Node):
    def __init__(self):
        super().__init__('rmf_demos_door_adapter')

        address = self.declare_parameter('manager_address', 'localhost').value
        port = self.declare_parameter('manager_port', 5002).value
        self.door_api = DoorAPI(address, port, self.get_logger())
        self.doors = set()

        self.door_state_pub = self.create_publisher(
            DoorState,
            'door_states',
            qos_profile=qos_profile_system_default)
        self.door_request_sub = self.create_subscription(
            DoorRequest,
            'door_requests',
            self.door_request_callback,
            qos_profile=qos_profile_system_default)
        self.pub_state_timer = self.create_timer(1.0, self.publish_states)
        self.get_logger().info('Running DemoDoorAdapter')

    def _door_state(self, door_name) -> Optional[DoorState]:
        new_state = DoorState()
        new_state.door_time = self.get_clock().now().to_msg()
        new_state.door_name = door_name

        door_mode = self.door_api.door_mode(door_name)
        if door_mode is None:
            self.get_logger().error('Unable to retrieve door mode')
            return None

        new_state.current_mode.value = door_mode
        return new_state

    def publish_states(self):
        self.doors = self.door_api.get_door_names()
        for door_name in self.doors:
            door_state = self._door_state(door_name)
            if door_state is None:
                continue
            self.door_state_pub.publish(door_state)

    def door_request_callback(self, msg):
        if msg.door_name not in self.doors:
            return

        if msg.requested_mode.value == DoorMode.MODE_OPEN:
            self.door_api.open_door(msg.door_name)

        elif msg.requested_mode.value == DoorMode.MODE_CLOSED:
            self.door_api.close_door(msg.door_name)


def main(argv=sys.argv):
    rclpy.init()
    node = DemoDoorAdapter()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
