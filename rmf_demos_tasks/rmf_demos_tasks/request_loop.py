#!/usr/bin/env python3

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
"""Request a loop."""

import argparse
import sys
import time
import uuid

import rclpy

from rmf_task_msgs.msg import Loop


def create_parser():
    """Create command line parser for request_loop."""
    parser = argparse.ArgumentParser(
        description='Request a robot fleet to perform loops between waypoints.'
    )
    parser.add_argument(
        '-s', '--start',
        required=True,
        type=str,
        help='Start waypoint'
    )
    parser.add_argument(
        '-f', '--finish',
        required=True,
        type=str,
        help='Finish waypoint'
    )
    parser.add_argument(
        '-n',
        '--num',
        help='Number of loops to perform',
        type=int,
        default=1,
    )
    parser.add_argument(
        '-i', '--task-id',
        help='Task ID',
        default='',
        type=str
    )
    parser.add_argument(
        '-r', '--robot-type',
        required=True,
        type=str,
        help='Fleet name'
    )
    parser.add_argument(
        '--delay',
        help='Number of secs to wait before sending out the request',
        default=0,
        type=int,
    )
    return parser


class LoopRequester:
    """Request a loop."""

    def __init__(self, args_or_argv=None):
        """Request a loop."""
        if args_or_argv is None:
            args_or_argv = sys.argv[1:]

        if isinstance(args_or_argv, argparse.Namespace):
            args = args_or_argv
        elif isinstance(args_or_argv, list):
            parser = create_parser()
            if len(args_or_argv) > 0 and not args_or_argv[0].startswith('-'):
                args = parser.parse_args(args_or_argv[1:])
            else:
                args = parser.parse_args(args_or_argv)
        else:
            parser = create_parser()
            args = parser.parse_args()

        self.start_wp = args.start
        self.finish_wp = args.finish
        self.num_loops = args.num
        self.task_id = args.task_id
        self.robot_type = args.robot_type
        self.num_sec_delay = args.delay

        self.node = rclpy.create_node('loop_requester_node')
        self.publisher = self.node.create_publisher(Loop, 'loop_requests', 10)

    def main(self):
        """Request a loop."""
        request = Loop()
        request.robot_type = self.robot_type
        request.start_name = self.start_wp
        request.finish_name = self.finish_wp
        request.num_loops = self.num_loops
        request.task_id = (
            self.task_id if self.task_id else 'loop#' + str(uuid.uuid1())
        )

        if self.num_sec_delay > 0:
            time.sleep(self.num_sec_delay)
        self.publisher.publish(request)
        time.sleep(0.5)

        self.node.get_logger().info(
            'Loop request between {} and {}, submitted to {} robot '
            'fleet'.format(self.start_wp, self.finish_wp, self.robot_type)
        )
        self.node.destroy_node()
        rclpy.shutdown()
        return 0


def main(argv=sys.argv):
    """Request a loop."""
    args_without_ros = rclpy.utilities.remove_ros_args(argv)
    parser = create_parser()
    args = parser.parse_args(args_without_ros[1:])

    rclpy.init(args=argv)
    try:
        loop_requester = LoopRequester(args)
        return loop_requester.main()
    except KeyboardInterrupt:
        return 1
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main(sys.argv))
