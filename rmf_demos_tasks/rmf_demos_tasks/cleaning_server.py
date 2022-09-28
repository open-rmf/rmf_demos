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
import rclpy
import argparse
import yaml
import time
from rclpy.node import Node

from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy as History
from rclpy.qos import QoSDurabilityPolicy as Durability
from rclpy.qos import QoSReliabilityPolicy as Reliability

from rmf_fleet_msgs.msg import CleanTaskSummary, CleanTask, \
    CleanTaskParameter, Location


class CleaningServer(Node):
    """
    The CleaningServer provides the coordinates for each clean zone
    in the respective demo worlds by publishing a CleanTaskSummary
    when launched.
    """

    def __init__(self, config_yaml):
        super().__init__('cleaning_server')
        self.get_logger().info(f'Starting cleaning server...')
        self.config_yaml = config_yaml

        transient_qos = QoSProfile(
            history=History.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1,
            reliability=Reliability.RMW_QOS_POLICY_RELIABILITY_RELIABLE,
            durability=Durability.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL)
        self.cleaning_server_publisher = self.create_publisher(
            CleanTaskSummary, 'clean_task_summary', qos_profile=transient_qos)

        # Populate the cleaning service msg
        clean_task_summary = CleanTaskSummary()
        for fleet_name, cleaning_info in self.config_yaml.items():
            clean_task = CleanTask()
            clean_task.fleet_name = fleet_name
            for clean_task_name, clean_waypoints in cleaning_info.items():
                param = CleanTaskParameter()
                param.name = clean_task_name
                for point in clean_waypoints['path']:
                    location = Location()
                    location.x = point[0]
                    location.y = point[1]
                    location.yaw = point[2]
                    location.level_name = clean_waypoints['level_name']
                    param.path.append(location)
                clean_task.params.append(param)
            clean_task_summary.clean_tasks.append(clean_task)
        time.sleep(2)
        self.cleaning_server_publisher.publish(clean_task_summary)


def main(argv=sys.argv):
    rclpy.init(args=argv)
    args_without_ros = rclpy.utilities.remove_ros_args(argv)
    parser = argparse.ArgumentParser(
        prog='cleaning_server',
        description='Start cleaning service')
    parser.add_argument("-c", "--config", type=str, required=True,
                        help="Path to config file")
    args = parser.parse_args(args_without_ros[1:])

    config = args.config

    with open(config, 'r') as f:
        config_yaml = yaml.safe_load(f)

    cleaning_server = CleaningServer(config_yaml)
    rclpy.spin(cleaning_server)
    rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv)
