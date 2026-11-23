#!/usr/bin/env python3

# Copyright 2025 Open Source Robotics Foundation, Inc.
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
"""Publish an emergency signal."""

import argparse
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy as Durability
from rclpy.qos import QoSHistoryPolicy as History
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy as Reliability

from rmf_fleet_msgs.msg import EmergencySignal


def str2bool(v):
    """Convert a string to a boolean."""
    if isinstance(v, bool):
        return v
    if v.lower() in ('yes', 'true', 't', 'y', '1'):
        return True
    elif v.lower() in ('no', 'false', 'f', 'n', '0'):
        return False
    raise argparse.ArgumentTypeError(
        f"Boolean value expected, got '{v}' (use true/false, yes/no, 1/0)."
    )


def create_parser():
    """Create command line parser for emergency_signal."""
    parser = argparse.ArgumentParser(
        description='Publish an emergency signal to robot fleets in Open-RMF.'
    )
    parser.add_argument(
        'is_emergency',
        type=str2bool,
        help='Emergency message to publish (true or false)',
    )
    parser.add_argument(
        '-F',
        '--fleets',
        type=str,
        help='Fleets to trigger (default: all fleets)',
        nargs='+',
    )
    parser.add_argument(
        '--once',
        action='store_true',
        help='Publish the emergency signal once and exit',
    )
    parser.add_argument(
        '--timeout',
        type=float,
        default=None,
        help='Timeout in seconds to continue publishing before exiting',
    )
    return parser


class EmergencySignalPublisher(Node):
    """Emergency signal publisher."""

    def __init__(self, args):
        """Initialize the emergency signal publisher."""
        super().__init__('emergency_signal_publisher')
        self.args = args

        qos_profile = QoSProfile(
            history=History.KEEP_LAST,
            depth=100,
            durability=Durability.TRANSIENT_LOCAL,
            reliability=Reliability.RELIABLE,
        )
        self.publisher = self.create_publisher(
            EmergencySignal, 'emergency_signal', qos_profile
        )

        self.msg = EmergencySignal()
        self.msg.is_emergency = self.args.is_emergency
        self.get_logger().info(
            f'Publishing emergency signal: {self.msg.is_emergency}'
        )
        if self.args.fleets is not None:
            self.msg.fleet_names = self.args.fleets
            self.get_logger().info(f'Fleets: {self.msg.fleet_names}')
        else:
            self.get_logger().info(
                'No fleets specified, sending to all fleets'
            )

        if self.args.once:
            self.publisher.publish(self.msg)
            self.get_logger().info('Emergency signal published once.')
        else:
            timer_period = 1.0  # seconds
            self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        """Timer callback to publish emergency signal."""
        self.publisher.publish(self.msg)
        self.get_logger().info('Publishing emergency signal')


###############################################################################


def main(argv=sys.argv):
    """Publish an emergency signal."""
    args_without_ros = rclpy.utilities.remove_ros_args(argv)
    parser = create_parser()
    args = parser.parse_args(args_without_ros[1:])

    rclpy.init(args=argv)
    signal_publisher = EmergencySignalPublisher(args)

    if not args.once:
        if args.timeout is not None and args.timeout > 0:
            timeout_future = rclpy.Future()

            def on_timeout():
                timeout_future.set_result(True)

            _ = signal_publisher.create_timer(args.timeout, on_timeout)
            rclpy.spin_until_future_complete(signal_publisher, timeout_future)
        else:
            rclpy.spin(signal_publisher)

    signal_publisher.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
