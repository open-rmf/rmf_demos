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

import sys
from time import sleep
import uuid

import rclpy
from rmf_lift_msgs.msg import LiftRequest


def print_instructions():
    print(
        'Invalid number of arguments, please pass in lift_name, desired ',
        'level and door state after the script in that order, only supports',
        "'open' or 'closed'.",
    )
    print(
        'For example:\n',
        '  request_lift Lift1 L1 open\n',
        '  request_lift Lift2 L3 closed',
    )


def main(argv=sys.argv):
    rclpy.init(args=argv)

    if len(argv) != 4:
        print_instructions()
        return 1

    request = LiftRequest()

    request.lift_name = argv[1]
    request.destination_floor = argv[2]
    request.session_id = 'lift_request#' + str(uuid.uuid1())

    if argv[3] == 'open':
        request.door_state = 2
    elif argv[3] == 'closed':
        request.door_state = 0
    else:
        print_instructions()
        return 1

    node = rclpy.create_node('lift_request_publisher')
    publisher = node.create_publisher(LiftRequest, '/lift_requests', 10)

    for _ in range(5):
        publisher.publish(request)
        sleep(0.5)

    rclpy.shutdown()

    print(
        f'Sent 5 messages at 2 Hz requesting lift: {argv[1]}, ',
        f'to floor: {argv[2]}, ',
        f'with door: {argv[3]}.',
    )


if __name__ == '__main__':
    main(sys.argv)
