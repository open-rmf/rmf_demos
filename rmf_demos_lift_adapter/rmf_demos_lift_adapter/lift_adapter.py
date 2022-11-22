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
import yaml
import argparse
from typing import Optional
from yaml import YAMLObject

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from rmf_lift_msgs.msg import LiftState, LiftRequest

from .LiftAPI import LiftAPI

"""
    The DemoLiftAdapter is a node which provide updates to Open-RMF, as well
    as handle incoming requests to control the integrated lift, by calling the
    implemented functions in LiftAPI.
"""


class DemoLiftAdapter(Node):
    def __init__(self, args, config: YAMLObject):
        super().__init__('rmf_demos_lift_adapter')

        self.lift_states = {}
        self.lift_requests = {}
        self.lift_config = config
        self.lift_api = LiftAPI(self.lift_config, self.get_logger())
        for lift in config['lifts']:
            self.lift_requests[lift] = None
            self.lift_states[lift] = self._lift_state(lift)

        self.lift_state_pub = self.create_publisher(
            LiftState,
            'lift_states',
            qos_profile=qos_profile_system_default)
        self.lift_request_sub = self.create_subscription(
            LiftRequest,
            'lift_requests',
            self.lift_request_callback,
            qos_profile=qos_profile_system_default)
        self.update_timer = self.create_timer(0.5, self.update_callback)
        self.pub_state_timer = self.create_timer(1.0, self.publish_states)
        self.get_logger().info('Running DemoLiftAdapter')

    def update_callback(self):
        new_states = {}
        for lift_name, lift_state in self.lift_states.items():
            new_state = self._lift_state(lift_name)
            new_states[lift_name] = new_state
            if new_state is None:
                self.get_logger().error(
                    f'Unable to get new state from lift {lift_name}')
                continue

            lift_request = self.lift_requests[lift_name]
            # No request to consider
            if lift_request is None:
                continue

            # If all is done, set request to None
            if lift_request.destination_floor ==\
                    new_state.current_floor and\
                    new_state.door_state == LiftState.DOOR_OPEN:
                lift_request = None
        self.lift_states = new_states

    def _lift_state(self, lift_name) -> Optional[LiftState]:
        new_state = LiftState()
        new_state.lift_time = self.get_clock().now().to_msg()
        new_state.lift_name = lift_name

        lift_state = self.lift_api.lift_state(lift_name)
        if lift_state is None:
            self.get_logger().error('Unable to retrieve lift state')
            return None

        new_state.available_floors = lift_state.available_floors
        new_state.current_floor = lift_state.current_floor
        new_state.destination_floor = lift_state.destination_floor
        new_state.door_state = lift_state.door_state
        new_state.motion_state = lift_state.motion_state

        new_state.available_modes = [LiftState.MODE_HUMAN, LiftState.MODE_AGV]
        new_state.current_mode = LiftState.MODE_AGV

        lift_request = self.lift_requests[lift_name]
        if lift_request is not None:
            if lift_request.request_type ==\
                    LiftRequest.REQUEST_END_SESSION:
                new_state.session_id = ''
            else:
                new_state.session_id = lift_request.session_id
        return new_state

    def publish_states(self):
        for lift_name, lift_state in self.lift_states.items():
            if lift_state is None:
                self.get_logger().info('No lift state received for lift '
                                       f'{lift_name}')
                continue
            self.lift_state_pub.publish(lift_state)

    def lift_request_callback(self, msg):
        if msg.lift_name not in self.lift_states:
            return

        lift_state = self.lift_states[msg.lift_name]
        if lift_state is not None and\
                msg.destination_floor not in lift_state.available_floors:
            self.get_logger().info(
                'Floor {} not available.'.format(msg.destination_floor))
            return

        if not self.lift_api.command_lift(msg.lift_name, msg.destination_floor, msg.door_state):
            self.get_logger().error(
                f'Failed to send lift to {msg.destination_floor}.')
            return

        self.get_logger().info(f'Requested lift {msg.lift_name} '
                               f'to {msg.destination_floor}.')
        self.lift_requests[msg.lift_name] = msg


def main(argv=sys.argv):
    args_without_ros = rclpy.utilities.remove_ros_args(argv)
    parser = argparse.ArgumentParser(
        prog='rmf_demos_lift_adapter',
        description='RMF Demos lift adapter')
    parser.add_argument('-c', '--config', required=True, type=str)
    args = parser.parse_args(args_without_ros[1:])

    with open(args.config, 'r') as f:
        config = yaml.safe_load(f)

    rclpy.init()
    node = DemoLiftAdapter(args, config)
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
