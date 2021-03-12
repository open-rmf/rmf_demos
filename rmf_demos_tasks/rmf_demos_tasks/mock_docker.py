
# Copyright 2020 Open Source Robotics Foundation, Inc.
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
from rclpy.node import Node
from rclpy.time import Time

from rclpy.qos import qos_profile_system_default
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

    # if abs(l1.yaw - l0.yaw) > 10.0*math.pi/180.0:
    #     return False

    return True


class MockDocker(Node):

    def __init__(self, config_yaml):
        super().__init__('mock_docker')
        print(f"Greetings, I am mock docker")
        self.config_yaml = config_yaml
        self.path_request_publisher = self.create_publisher(
            PathRequest, 'robot_path_requests', 1)

        self.mode_request_publisher = self.create_publisher(
            ModeRequest, 'robot_mode_requests', 1)

        self.mode_request_subscription = self.create_subscription(
            ModeRequest, 'robot_mode_requests', self.mode_request_cb, 1)

        self.robot_state_subscription = self.create_subscription(
            RobotState, 'robot_state', self.robot_state_cb, 1)

        transient_qos = QoSProfile(
            history=History.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1,
            reliability=Reliability.RMW_QOS_POLICY_RELIABILITY_RELIABLE,
            durability=Durability.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL)
        self.dock_summary_publisher = self.create_publisher(
            DockSummary, 'dock_summary', qos_profile=transient_qos)

        # self.level_name = "B2"

        # self.dock_map = {}
        # # TODO dock_map should contain a nested dict for
        # # each fleet's docking points
        # for key, p_array in self.config_yaml["ecobot"].items():
        #     location_list = []
        #     for p in p_array:
        #         p_loc = make_location(p, self.level_name)
        #         location_list.append(p_loc)
        #     self.dock_map[key] = location_list

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
                # This is a hack. The cleaning task will ensure the robot ends
                # up at the finish waypoint. The graph already containts these
                # waypoints
                param.finish = param.start + "_start"
                for point in dock_waypoints["path"]:
                    location = make_location(point, dock_waypoints["level_name"])
                    param.path.append(location)
                dock.params.append(param)
                dock_sub_map[dock_name] = param.path
            dock_summary.docks.append(dock)
            self.dock_map[fleet_name] = dock_sub_map
        self.dock_summary_publisher.publish(dock_summary)

    def mode_request_cb(self, msg: ModeRequest):
        if msg.mode.mode != RobotMode.MODE_DOCKING:
            return

        if not msg.parameters:
            print(f'Missing docking name for docking request!')
            return

        if msg.parameters[0].name != 'docking':
            print(f'Unexpected docking parameter [{msg.parameters[0]}]')
            return

        dock = self.dock_map.get(msg.fleet_name).get(msg.parameters[0].value)
        if not dock:
            print(f'Unknown dock name requested [{msg.parameters[0].value}]')
            return

        self.watching[msg.robot_name] = dock[-1]

        path_request = PathRequest()
        path_request.fleet_name = msg.fleet_name
        path_request.robot_name = msg.robot_name
        path_request.task_id = msg.task_id
        path_request.path = dock

        self.path_request_publisher.publish(path_request)

    def robot_state_cb(self, msg: RobotState):
        # self.level_name = msg.location.level_name
        remove_from_watching = []
        for robot_name, finish_location in self.watching.items():
            if robot_name != msg.name:
                continue

            if not close(finish_location, msg.location):
                continue

            mode_request = ModeRequest()
            # TODO(MXG): Getting fleet name like this is a hack
            mode_request.fleet_name = msg.name[:-2]
            mode_request.robot_name = msg.name
            mode_request.task_id = msg.task_id
            mode_request.mode.mode = RobotMode.MODE_MOVING
            for i in range(5):
                self.mode_request_publisher.publish(mode_request)
            remove_from_watching.append(robot_name)

        for robot in remove_from_watching:
            self.watching.pop(robot)


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
