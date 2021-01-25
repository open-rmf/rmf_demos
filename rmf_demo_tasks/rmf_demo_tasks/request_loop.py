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

import sys
import uuid
import time
import argparse

import rclpy
from rclpy.node import Node
from rmf_task_msgs.msg import Loop
from rmf_fleet_msgs.msg import FleetState


class LoopRequester:

    def __init__(self, argv=sys.argv):
        parser = argparse.ArgumentParser()
        parser.add_argument('-s', '--start', help='Start waypoint')
        parser.add_argument('-f', '--finish', help='Finish waypoint')
        parser.add_argument('-n', '--num', help='Number of loops to perform',
                            type=int, default=1)
        parser.add_argument('-i', '--task-id', help='Task ID', default='',
                            type=str)
        parser.add_argument('-r', '--robot-type', help='Fleet name')
        parser.add_argument(
            '--delay',
            help='Number of secs to wait before sending out the request',
            default=0, type=int)
        args = parser.parse_args(argv[1:])

        self.start_wp = args.start
        self.finish_wp = args.finish
        self.num_loops = args.num
        self.task_id = args.task_id
        self.robot_type = args.robot_type
        self.num_sec_delay = args.delay

        self.node = rclpy.create_node('loop_requester_node')
        self.publisher = self.node.create_publisher(Loop, 'loop_requests', 10)

    def main(self):
        request = Loop()
        request.robot_type = self.robot_type
        request.start_name = self.start_wp
        request.finish_name = self.finish_wp
        request.num_loops = self.num_loops
        request.task_id = self.task_id if self.task_id \
            else 'loop#' + str(uuid.uuid1())

        time.sleep(self.num_sec_delay)
        self.publisher.publish(request)
        time.sleep(0.5)
        rclpy.shutdown()

        self.node.get_logger().info(
          'Loop request between {} and {}, submitted to {} robot fleet'
          .format(self.start_wp, self.finish_wp, self.robot_type))


def main(argv=sys.argv):
    rclpy.init(args=sys.argv)
    args_without_ros = rclpy.utilities.remove_ros_args(sys.argv)

    loop_requester = LoopRequester(args_without_ros)
    loop_requester.main()


if __name__ == '__main__':
    main(sys.argv)
