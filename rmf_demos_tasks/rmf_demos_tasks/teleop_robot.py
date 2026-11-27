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
"""Teleop a robot."""

import argparse
import sys
import uuid

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy as Durability
from rclpy.qos import QoSHistoryPolicy as History
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy as Reliability

from rmf_fleet_msgs.msg import Location
from rmf_fleet_msgs.msg import PathRequest

###############################################################################


def parse_point(point_str):
    """Parse and validate a comma-separated x,y,yaw coordinate point."""
    pts = point_str.split(',')
    if len(pts) != 3:
        raise argparse.ArgumentTypeError(
            f"Point '{point_str}' must have 3 comma-separated values "
            '{x,y,yaw}'
        )
    try:
        return float(pts[0]), float(pts[1]), float(pts[2])
    except ValueError:
        raise argparse.ArgumentTypeError(
            f"Point '{point_str}' contains non-numeric coordinates"
        )


def create_parser():
    """Create command line parser for teleop_robot."""
    parser = argparse.ArgumentParser(
        description='Publish a teleop path request to a robot in Open-RMF.'
    )
    parser.add_argument(
        '-F', '--fleet', required=True, type=str, help='Fleet name'
    )
    parser.add_argument(
        '-R', '--robot', required=True, type=str, help='Robot name'
    )
    parser.add_argument(
        '-p',
        '--points',
        required=True,
        nargs='+',
        type=parse_point,
        help='Coordinate points [x,y,yaw] for teleop (e.g. 1.0,2.0,0.0)',
    )
    parser.add_argument(
        '-m', '--map', required=True, type=str, help='Map name'
    )
    parser.add_argument(
        '--timeout',
        type=float,
        default=5.0,
        help='Timeout in seconds to wait for publishing, default: 5.0',
    )
    return parser


class Requester(Node):
    """Teleop requester."""

    def __init__(self, args):
        """Initialize the requester."""
        super().__init__('teleop_publisher')
        self.args = args

        transient_qos = QoSProfile(
            history=History.KEEP_LAST,
            depth=1,
            reliability=Reliability.RELIABLE,
            durability=Durability.TRANSIENT_LOCAL,
        )
        self.pub = self.create_publisher(
            PathRequest, 'robot_path_requests', transient_qos
        )

        msg = PathRequest()
        msg.fleet_name = self.args.fleet
        msg.robot_name = self.args.robot
        msg.task_id = str(uuid.uuid1())
        for x, y, yaw in self.args.points:
            loc = Location()
            loc.x = x
            loc.y = y
            loc.yaw = yaw
            loc.level_name = self.args.map
            msg.path.append(loc)
        self.pub.publish(msg)


###############################################################################


def main(argv=sys.argv):
    """Teleop a robot."""
    args_without_ros = rclpy.utilities.remove_ros_args(argv)
    parser = create_parser()
    args = parser.parse_args(args_without_ros[1:])

    rclpy.init(args=argv)
    requester = Requester(args)

    timeout = rclpy.Future()

    def trigger_timeout():
        timeout.set_result(True)

    _ = requester.create_timer(args.timeout, trigger_timeout)

    rclpy.spin_until_future_complete(requester, timeout)
    requester.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
