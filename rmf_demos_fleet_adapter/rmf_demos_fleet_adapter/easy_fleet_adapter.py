# Copyright 2021 Open Source Robotics Foundation, Inc.
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
import argparse
import yaml
import time
import threading
import datetime

import rclpy
import rclpy.node
from rclpy.parameter import Parameter

import rmf_adapter as adpt
import rmf_adapter.plan as plan

from .configuration import get_configuration

from functools import partial

from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy as History
from rclpy.qos import QoSDurabilityPolicy as Durability
from rclpy.qos import QoSReliabilityPolicy as Reliability
from rclpy.qos import qos_profile_system_default

from .RobotClientAPI import RobotAPI

# ------------------------------------------------------------------------------
# Helper functions
# ------------------------------------------------------------------------------


class FleetAdapter:

    def __init__(self, config_yaml, nav_graph_path, node, use_sim_time):
        # global counter for command ids
        self.next_id = 0
        # Keep track of which map the robot is in
        self.last_map = {}
        self.cmd_ids = {}
        # Initialize robot API for this fleet
        fleet_config = config_yaml['rmf_fleet']
        prefix = 'http://' + fleet_config['fleet_manager']['ip'] + \
                ':' + str(fleet_config['fleet_manager']['port'])
        self.api = RobotAPI(
            prefix,
            fleet_config['fleet_manager']['user'],
            fleet_config['fleet_manager']['password'])

        configuration = get_configuration(config_yaml, nav_graph_path, node)
        self.adapter = self.initialize_fleet(configuration, config_yaml['robots'], node, use_sim_time)

    def initialize_fleet(self, configuration, robots_yaml, node, use_sim_time):
        # Make the easy full control
        easy_full_control = adpt.EasyFullControl.make(configuration)

        if use_sim_time:
            easy_full_control.node.use_sim_time()

        def _goal_completed(robot_name, remaining_time, request_replan):
            request_replan = self.api.requires_replan(robot_name)
            remaining_time = self.api.navigation_remaining_duration(robot_name, self.cmd_ids[robot_name])
            return self.api.process_completed(robot_name, self.cmd_ids[robot_name])

        def _robot_state(robot_name):
            data = self.api.data(robot_name)
            if data is None or data['success'] is False:
                return None
            pos = data['data']['position']
            state = adpt.easy_full_control.RobotState(
                robot,
                robot_config['charger']['waypoint'],
                data['data']['map_name'],
                [pos['x'], pos['y'], pos['yaw']],
                data['data']['battery'])
            self.last_map[robot_name] = data['data']['map_name']
            return state

        def _navigate(robot_name, map_name, goal, update_handle):
            cmd_id = self.next_id
            self.next_id += 1
            self.cmd_ids[robot_name] = cmd_id
            self.api.navigate(robot_name, cmd_id, goal, map_name)
            node.get_logger().info(f"Navigating robot {robot_name}")
            return partial(_goal_completed, robot_name)

        def _stop(robot_name):
            cmd_id = self.next_id
            self.next_id += 1
            self.cmd_ids[robot_name] = cmd_id
            return self.api.stop(robot_name, cmd_id)

        def _dock(robot_name, dock_name, update_handle):
            cmd_id = self.next_id
            self.next_id += 1
            self.cmd_ids[robot_name] = cmd_id
            self.api.start_process(robot_name, cmd_id, dock_name, self.last_map[robot_name])
            return partial(_goal_completed, robot_name)

        def _action_executor(robot_name: str,
                             category: str,
                             description: dict,
                             execution: adpt.robot_update_handle.ActionExecution):
            pass

        # Add the robots
        for robot in robots_yaml:
            node.get_logger().info(f'Found robot {robot}')
            robot_config = robots_yaml[robot]['rmf_config']
            success = False
            while success is False:
                state = _robot_state(robot)
                if state is None:
                    time.sleep(0.2)
                    continue
                success = True
                # Add robot to fleet
                easy_full_control.add_robot(
                    state,
                    partial(_robot_state, robot),
                    partial(_navigate, robot),
                    partial(_stop, robot),
                    partial(_dock, robot),
                    partial(_action_executor, robot))

        return easy_full_control


# ------------------------------------------------------------------------------
# Main
# ------------------------------------------------------------------------------
def main(argv=sys.argv):
    # Init rclpy and adapter
    rclpy.init(args=argv)
    adpt.init_rclcpp()
    args_without_ros = rclpy.utilities.remove_ros_args(argv)

    parser = argparse.ArgumentParser(
        prog="fleet_adapter",
        description="Configure and spin up the fleet adapter")
    parser.add_argument("-c", "--config_file", type=str, required=True,
                        help="Path to the config.yaml file")
    parser.add_argument("-n", "--nav_graph", type=str, required=True,
                        help="Path to the nav_graph for this fleet adapter")
    parser.add_argument("-sim", "--use_sim_time", action="store_true",
                        help='Use sim time, default: false')
    args = parser.parse_args(args_without_ros[1:])
    print(f"Starting fleet adapter...")

    config_path = args.config_file
    nav_graph_path = args.nav_graph

    # Load config and nav graph yamls
    with open(config_path, "r") as f:
        config_yaml = yaml.safe_load(f)

    # ROS 2 node for the command handle
    fleet_name = config_yaml['rmf_fleet']['name']
    node = rclpy.node.Node(f'{fleet_name}_command_handle')

    # Enable sim time for testing offline
    if args.use_sim_time:
        param = Parameter("use_sim_time", Parameter.Type.BOOL, True)
        node.set_parameters([param])

    adapter = FleetAdapter(
        config_yaml,
        nav_graph_path,
        node,
        args.use_sim_time)

    # Create executor for the command handle node
    rclpy_executor = rclpy.executors.SingleThreadedExecutor()
    rclpy_executor.add_node(node)

    # Start the fleet adapter
    rclpy_executor.spin()

    # Shutdown
    node.destroy_node()
    rclpy_executor.shutdown()
    rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv)
