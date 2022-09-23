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

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_system_default

from rmf_fleet_msgs.msg import PathRequest, Location


###############################################################################

class Requester(Node):

    def __init__(self, argv=sys.argv):
        super().__init__('teleop_publisher')
        parser = argparse.ArgumentParser()

        parser.add_argument('-F', '--fleet', required=True,
                            type=str, help='Fleet name')
        parser.add_argument('-R', '--robot', required=True,
                            help='Robot name', type=str)
        parser.add_argument('-p', '--points', required=True, nargs='+',
                            help='Coordinate points [x,y,yaw] for teleop',
                            type=str)
        parser.add_argument('-m', '--map', required=True,
                            help='Map name', type=str)

        self.args = parser.parse_args(argv[1:])

        self.pub = self.create_publisher(
          PathRequest, 'robot_path_requests',
          qos_profile=qos_profile_system_default)

        msg = PathRequest()
        msg.fleet_name = self.args.fleet
        msg.robot_name = self.args.robot
        msg.task_id = str(uuid.uuid1())
        for p in self.args.points:
            pts = p.split(',')
            assert len(pts) == 3, "each point should have 3 values {x,y,yaw}"
            loc = Location()
            loc.x = float(pts[0])
            loc.y = float(pts[1])
            loc.yaw = float(pts[2])
            loc.level_name = self.args.map
            msg.path.append(loc)
        self.pub.publish(msg)

###############################################################################


def main(argv=sys.argv):
    rclpy.init(args=sys.argv)
    args_without_ros = rclpy.utilities.remove_ros_args(sys.argv)

    requester = Requester(args_without_ros)
    rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv)
