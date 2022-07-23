
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
import rclpy
import math
import argparse
import yaml
import time
from rclpy.node import Node
from rclpy.time import Time

from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy as History
from rclpy.qos import QoSDurabilityPolicy as Durability
from rclpy.qos import QoSReliabilityPolicy as Reliability

from rmf_fleet_msgs.msg import ModeRequest, PathRequest, Location, \
    RobotState, RobotMode, DockSummary, Dock, DockParameter


def make_location(p, level_name):
    location = Location()
    location.x = p[0]
    location.y = p[1]
    location.yaw = p[2]
    location.level_name = level_name
    return location


def close(l0: Location, l1: Location):
    x_2 = (l1.x - l0.x) ** 2
    y_2 = (l1.y - l0.y) ** 2
    dist = math.sqrt(x_2 + y_2)
    if dist > 0.2:
        return False
    return True


class MockDocker(Node):
    """
    The MockDocker has two objectices
    1) Publish a DockSummary message with information on all the docking
    processes in a building. The fleet adapters rely on this message for
    task planning.
    2) [deprecated] To coordinate the docking motion of robots in simulation
    which are controlled by the deprecated full_control fleet adapter and the
    slotcar plugin. This is achieved by listening to the deprecated ModeRequest
    message published by the full_control adapter and publihsing the relevant
    PathRequest with waypoints from the configuration file. The node also
    monitors whether the robot has completed docking and informs the
    full_control adapter of the same. This approach is not reflective of how
    docking processes should be triggered on real robots. The relevant API
    call to the robot's fleet manager should be made to coordinate docking.
    """

    def __init__(self, config_yaml):
        super().__init__('mock_docker')
        self.get_logger().info(f'Greetings, I am mock docker')
        self.config_yaml = config_yaml
        self.path_request_publisher = self.create_publisher(
            PathRequest, 'robot_path_requests', 1)

        self.mode_request_publisher = self.create_publisher(
            ModeRequest, 'robot_mode_requests', 1)

        self.mode_request_subscription = self.create_subscription(
            ModeRequest, 'robot_mode_requests', self.mode_request_cb, 10)

        self.robot_state_subscription = self.create_subscription(
            RobotState, 'robot_state', self.robot_state_cb, 10)

        transient_qos = QoSProfile(
            history=History.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1,
            reliability=Reliability.RMW_QOS_POLICY_RELIABILITY_RELIABLE,
            durability=Durability.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL)
        self.dock_summary_publisher = self.create_publisher(
            DockSummary, 'dock_summary', qos_profile=transient_qos)

        # This is a dict of robots which are in docking mode
        self.watching = {}

        # Populate the dock_map and dock_summary msg
        dock_summary = DockSummary()
        self.dock_map = {}
        for fleet_name, docking_info in self.config_yaml.items():
            dock_sub_map = {}
            dock = Dock()
            dock.fleet_name = fleet_name
            for dock_name, dock_waypoints in docking_info.items():
                param = DockParameter()
                param.start = dock_name
                finish_waypoint = dock_waypoints.get("finish_waypoint")
                if finish_waypoint is None:
                    # for backwards compatibility
                    finish_waypoint = dock_name
                param.finish = finish_waypoint
                for point in dock_waypoints["path"]:
                    location = make_location(
                        point, dock_waypoints["level_name"])
                    param.path.append(location)
                dock.params.append(param)
                dock_sub_map[dock_name] = param.path
            dock_summary.docks.append(dock)
            self.dock_map[fleet_name] = dock_sub_map
        time.sleep(2)
        self.dock_summary_publisher.publish(dock_summary)

    def mode_request_cb(self, msg: ModeRequest):
        if msg.mode.mode != RobotMode.MODE_DOCKING:
            return

        if not msg.parameters:
            self.get_logger().warn(
                f'Missing docking name for docking request!')
            return

        if msg.parameters[0].name != 'docking':
            self.get_logger().warn(
                f'Unexpected docking parameter [{msg.parameters[0]}]')
            return

        fleet_name = self.dock_map.get(msg.fleet_name)
        if fleet_name is None:
            self.get_logger().warn(
                'Unknown fleet name requested [{msg.fleet_name}].')
            return

        dock = fleet_name.get(msg.parameters[0].value)
        if not dock:
            self.get_logger().warn(
                f'Unknown dock name requested [{msg.parameters[0].value}]')
            return

        self.get_logger().info(
            f'Received Docking Mode Request from [{msg.robot_name}]')
        path_request = PathRequest()
        path_request.fleet_name = msg.fleet_name
        path_request.robot_name = msg.robot_name
        path_request.task_id = msg.task_id
        path_request.path = dock
        self.watching[msg.robot_name] = path_request

        self.path_request_publisher.publish(path_request)

    def robot_state_cb(self, msg: RobotState):
        robot_name = msg.name
        if robot_name not in self.watching:
            return

        requested_path = self.watching[msg.name]
        finish_location = requested_path.path[-1]
        if not close(finish_location, msg.location):
            return

        # This is needed to acknowledge the slot car that a Docking Mode
        # is completed. Subsequently, this will update the robot_state and
        # inform the Fleet adapter that the robot has finished docking
        mode_request = ModeRequest()
        mode_request.fleet_name = requested_path.fleet_name
        mode_request.robot_name = requested_path.robot_name
        mode_request.task_id = requested_path.task_id
        mode_request.mode.mode = RobotMode.MODE_PAUSED
        self.mode_request_publisher.publish(mode_request)

        # Remove from watching, when it is no longer in Docking
        if msg.mode.mode != RobotMode.MODE_DOCKING:
            self.watching.pop(robot_name)
            self.get_logger().info(
                f'{robot_name} done with docking at {finish_location}')


def main(argv=sys.argv):
    rclpy.init(args=argv)
    args_without_ros = rclpy.utilities.remove_ros_args(argv)
    parser = argparse.ArgumentParser(
        prog="mock_docker",
        description="Configure and start mock_docker node")
    parser.add_argument("-c", "--config", type=str, required=True,
                        help="Path to config file")
    args = parser.parse_args(args_without_ros[1:])

    config = args.config

    with open(config, "r") as f:
        config_yaml = yaml.safe_load(f)

    mock_docker = MockDocker(config_yaml)
    rclpy.spin(mock_docker)
    rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv)
