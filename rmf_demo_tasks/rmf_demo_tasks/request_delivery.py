
import argparse
import sys
from time import sleep
import uuid

import rclpy

from rmf_task_msgs.msg import Delivery

def main(argv = sys.argv):
    rclpy.init(args=argv)

    '''
    # Example request:
    task_id: randomid_001
    items: [itemA, itemB....]
    pickup_place_name: cssd_room
    pickup_behavior:
    - name: dispenser
    - parameters: [request_guid: xxx, target_guid:cssdbot, transporter_type:mir]
    dropoff_place_name: ot_prep_room
    dropoff_behavior:
    - name: dispenser
    - parameters: [request_guid: yyy, target_guid:otbot, transporter_type:mir]
    '''

    parser = argparse.ArgumentParser()
    parser.add_argument('-p', '--pickup', default='pantry', help='Start waypoint')
    parser.add_argument('-d', '--dropoff', default='hardware_2', help='Finish waypoint')
    parser.add_argument('-i', '--task-id', help='Task ID', default='', type=str)
    parser.add_argument('-r', '--robot-type', help='Type of robot', default='magni')

    args = parser.parse_args(argv[1:])

    node = rclpy.create_node('loop_request_publisher')
    publisher = node.create_publisher(Delivery, 'delivery_requests', 10)

    sleep(0.5)

    request = Delivery()
    if args.task_id:
        request.task_id = args.task_id
    else:
        request.task_id = 'delivery#' + str(uuid.uuid1())
    request.pickup_place_name = args.pickup
    request.dropoff_place_name = args.dropoff

    for _ in range(5):
        publisher.publish(request)
        sleep(0.5)

    rclpy.shutdown()


    print(f'Delivery request submitted to {args.robot_type}')

if __name__ == '__main__':
    main(sys.argv)
