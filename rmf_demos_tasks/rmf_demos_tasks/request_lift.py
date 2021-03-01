import argparse
import sys
from time import sleep
import uuid

import rclpy

from rmf_lift_msgs.msg import LiftRequest


def print_instructions():
    print("Invalid number of arguments, please pass in lift_name, desired ",
          "level and door state after the script in that order, only supports",
          "'open' or 'closed'.")
    print('For example:\n',
          '  request_lift Lift1 L1 open\n',
          '  request_lift Lift2 L3 closed')


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

    print(f'Sent 5 messages at 2 Hz requesting lift: {argv[1]}, ',
          f'to floor: {argv[2]}, ',
          f'with door: {argv[3]}.')


if __name__ == '__main__':
    main(sys.argv)
