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
import argparse
import yaml
import time
import threading

import rclpy
import rclpy.node
from rclpy.parameter import Parameter

import rmf_adapter as adpt

from .RobotClientAPI import RobotAPI

# ------------------------------------------------------------------------------
# Helper functions
# ------------------------------------------------------------------------------


def add_easy_fleet_robots(node, easy_adapter, config_yaml):
    # Initialize robot API for this fleet
    fleet_config = config_yaml['rmf_fleet']
    prefix = 'http://' + fleet_config['fleet_manager']['ip'] + \
             ':' + str(fleet_config['fleet_manager']['port'])
    api = RobotAPI(
        prefix,
        fleet_config['fleet_manager']['user'],
        fleet_config['fleet_manager']['password'])

    # List of robot names
    missing_robots = config_yaml['robots']

    # Assign callbacks to each robot of the fleet
    def _assign_callbacks(robot_name: str, start_pose: list):

        # Define get_position callback for this robot
        def _get_position():
            current_status = api.data(robot_name)
            if not current_status:
                return
            pose = current_status['data']['position']
            battery_soc = current_status['data']['battery']
            map_name = current_status['data']['map_name']
            replan = api.requires_replan(robot_name)

            position = adpt.easy_full_control_handle.Position()
            position.position = [pose['x'], pose['y'], pose['yaw']]
            position.battery_percent = battery_soc
            position.map_name = map_name
            position.replan = replan
            return position

        # Use this callback to check whether the robot completed process
        def _check_completed(cmd_id: int):
            resp = api.process_completed(robot_name, cmd_id)
            return resp

        # Define navigate callback for this robot
        def _navigate(target: adpt.easy_full_control_handle.Target):
            resp = api.navigate(robot_name, target.cmd_id, target.pose,
                                target.map_name, target.speed_limit)
            if resp:
                return _check_completed
            else:
                return

        # Define dock callback for this robot
        def _dock(dock_name: str, cmd_id: int):
            resp = api.start_process(
                robot_name, cmd_id, dock_name, map_name='')
            if resp:
                return _check_completed
            else:
                return

        # Define stop function for this robot
        def _stop(cmd_id: int):
            resp = api.stop(robot_name, cmd_id)
            return resp

        # Define action executor callback for this robot
        def _action_executor(category: str,
                             description: dict,
                             execution:
                             adpt.robot_update_handle.ActionExecution):
            pass  # do nothing for now

        success = easy_adapter.add_robot(robot_name,
                                         start_pose,
                                         _get_position,
                                         _navigate,
                                         _dock,
                                         _stop,
                                         _action_executor)
        return success

    def _add_fleet_robots():
        while len(missing_robots) > 0:
            time.sleep(0.2)

            for robot_name in list(missing_robots.keys()):
                node.get_logger().debug(f'Connecting to robot: {robot_name}')
                data = api.data(robot_name)
                if data is None:
                    continue
                if data['success']:
                    # We use the robot's current position to compute plan start
                    position = api.position(robot_name)
                    if position is None:
                        node.get_logger().info(
                            f'Failed to get initial position of {robot_name}')
                        continue
                    start_pose = \
                        [position[0], position[1], position[2]]

                    success = _assign_callbacks(robot_name, start_pose)

                    if success:
                        node.get_logger().info(
                            f'Successfully added new robot: {robot_name}')
                    else:
                        node.get_logger().error(
                            f'Failed to initialize robot: {robot_name}')

                    del missing_robots[robot_name]

                else:
                    node.get_logger().debug(
                        f'{robot_name} not found, trying again...')

        return

    add_robots = threading.Thread(target=_add_fleet_robots, args=())
    add_robots.start()


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

    # Set server uri
    node.declare_parameter('server_uri', rclpy.Parameter.Type.STRING)
    server_uri = node.get_parameter(
        'server_uri').get_parameter_value().string_value
    if server_uri == "":
        server_uri = None

    # Make an adapter instance
    adapter = adpt.Adapter.make(f'{fleet_name}_fleet_adapter')
    if args.use_sim_time:
        adapter.node.use_sim_time()
    assert adapter, ("Unable to initialize fleet adapter. Please ensure "
                     "RMF Schedule Node is running")
    adapter.start()
    time.sleep(1.0)

    # Set up Configuration and make EasyFullControl fleet
    adapter_config = adpt.easy_full_control_handle.Configuration(
        config_path,
        nav_graph_path,
        server_uri)

    easy_adapter = adpt.EasyFullControl.make(adapter_config, adapter)
    assert easy_adapter, ("Unable to initialize easy fleet adapter.")

    # Add robots to fleet adapter
    add_easy_fleet_robots(node, easy_adapter, config_yaml)

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
