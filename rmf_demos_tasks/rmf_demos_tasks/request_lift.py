# Copyright 2024 Open Source Robotics Foundation, Inc.
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
"""Request a lift to move somewhere."""

import argparse
import sys
from time import sleep
import uuid

import rclpy

from rmf_lift_msgs.msg import LiftRequest


def create_parser():
    """Create command line parser for request_lift."""
    parser = argparse.ArgumentParser(
        description='Request a lift to move to a destination floor and '
                    'set its door state.'
    )
    parser.add_argument(
        'lift_name',
        type=str,
        nargs='?',
        default=None,
        help='Name of the lift (e.g. Lift1)',
    )
    parser.add_argument(
        'destination_floor',
        type=str,
        nargs='?',
        default=None,
        help='Target destination floor level (e.g. L1)',
    )
    parser.add_argument(
        'door_state',
        type=str,
        nargs='?',
        default=None,
        choices=['open', 'closed'],
        help="Target door state ('open' or 'closed')",
    )
    parser.add_argument(
        '-l', '--lift',
        type=str,
        dest='opt_lift',
        help='Name of the lift (alternative to positional argument)',
    )
    parser.add_argument(
        '-f', '--floor',
        type=str,
        dest='opt_floor',
        help='Target destination floor level '
             '(alternative to positional argument)',
    )
    parser.add_argument(
        '-d', '--door',
        type=str,
        dest='opt_door',
        choices=['open', 'closed'],
        help="Target door state ('open' or 'closed')",
    )
    return parser


def main(argv=sys.argv):
    """Request a lift to move somewhere."""
    args_without_ros = rclpy.utilities.remove_ros_args(argv)
    parser = create_parser()
    args = parser.parse_args(args_without_ros[1:])

    lift_name = args.opt_lift if args.opt_lift is not None else args.lift_name
    floor = (
        args.opt_floor if args.opt_floor is not None
        else args.destination_floor
    )
    door = args.opt_door if args.opt_door is not None else args.door_state

    if lift_name is None or floor is None:
        parser.error('Both lift name and destination floor must be specified.')

    if door is None:
        door = 'open'

    request = LiftRequest()
    request.lift_name = lift_name
    request.destination_floor = floor
    request.session_id = 'lift_request#' + str(uuid.uuid1())

    if door == 'open':
        request.door_state = LiftRequest.DOOR_OPEN
    elif door == 'closed':
        request.door_state = LiftRequest.DOOR_CLOSED

    rclpy.init(args=argv)
    node = rclpy.create_node('lift_request_publisher')
    publisher = node.create_publisher(LiftRequest, '/lift_requests', 10)

    for _ in range(5):
        publisher.publish(request)
        sleep(0.5)

    node.destroy_node()
    rclpy.shutdown()

    print(
        f'Sent 5 messages at 2 Hz requesting lift: {lift_name}, '
        f'to floor: {floor}, '
        f'with door: {door}.'
    )
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
