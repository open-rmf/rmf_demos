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

import argparse
import asyncio
import sys

import rclpy
import rclpy.node
from rclpy.qos import QoSDurabilityPolicy as Durability
from rclpy.qos import QoSHistoryPolicy as History
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy as Reliability
import rmf_adapter
import rmf_adapter.geometry as geometry
import rmf_adapter.vehicletraits as traits

from rmf_fleet_msgs.msg import LaneRequest


def main(argv=sys.argv):
    rclpy.init(args=argv)
    args_without_ros = rclpy.utilities.remove_ros_args(argv)

    parser = argparse.ArgumentParser(
        prog='manage_lane',
        description='Open or close a lane between two waypoints'
    )
    parser.add_argument(
        'request',
        choices=['open', 'close'],
        help='Whether the lane should be open or closed',
    )
    parser.add_argument(
        'from_waypoint',
        type=str,
        help='Name of the waypoint at the start of the lane',
    )
    parser.add_argument(
        'to_waypoint',
        type=str,
        help='Name of the waypoint at the end of the lane',
    )
    parser.add_argument(
        '-b',
        '--bidir',
        action='store_true',
        help='Apply the state change in both directions (bidirectionally)'
    )
    parser.add_argument(
        '-F',
        '--fleet',
        type=str,
        default='',
        help=(
            'Which fleet should the lane be closed for. '
            'Empty string (default) indicates all fleets.'
        ),
    )
    parser.add_argument(
        '-n',
        '--nav_graph',
        type=str,
        required=True,
        help='Path to the nav_graph that contains the lane',
    )

    args = parser.parse_args(args_without_ros[1:])

    t = traits.VehicleTraits(
        linear=traits.Limits(1.0, 1.0),
        angular=traits.Limits(1.0, 1.0),
        profile=traits.Profile(geometry.make_final_convex_circle(1.0)),
    )

    nav_graph = rmf_adapter.graph.parse_graph(args.nav_graph, t)

    from_waypoint = nav_graph.find_waypoint(args.from_waypoint)
    if from_waypoint is None:
        raise Exception(
            f'Unable to find waypoint [{args.from_waypoint}] in graph'
        )

    to_waypoint = nav_graph.find_waypoint(args.to_waypoint)
    if to_waypoint is None:
        raise Exception(
            f'Unable to find waypoint [{args.to_waypoint}] in graph'
        )

    lane_indices = []

    lane = nav_graph.lane_from(from_waypoint.index, to_waypoint.index)
    if lane is None:
        raise Exception(
            f'Unable to find a lane that connects [{args.from_waypoint}] '
            f'to [{args.to_waypoint}]'
        )

    lane_indices.append(lane.index)

    if args.bidir:
        reverse_lane = nav_graph.lane_from(
            to_waypoint.index, from_waypoint.index
        )
        if reverse_lane is not None:
            lane_indices.append(reverse_lane.index)

    request = LaneRequest()
    if args.request == 'open':
        request.open_lanes = lane_indices
    if args.request == 'close':
        request.close_lanes = lane_indices
    request.fleet_name = args.fleet

    node = rclpy.node.Node('manage_lane')

    transient_qos = QoSProfile(
        history=History.KEEP_LAST,
        depth=1,
        reliability=Reliability.RELIABLE,
        durability=Durability.TRANSIENT_LOCAL,
    )

    publisher = node.create_publisher(
        LaneRequest,
        'lane_closure_requests',
        qos_profile=transient_qos,
    )

    publisher.publish(request)

    rclpy_executor = rclpy.executors.SingleThreadedExecutor()
    rclpy_executor.add_node(node)

    print(f'Sending request:\n{request}')

    f = asyncio.Future()
    rclpy_executor.spin_until_future_complete(f, 5.0)
    # TODO(@mxgrey): Subscribe to lane closure topic and keep spinning until
    # we see these lanes open/closed


if __name__ == '__main__':
    main(sys.argv)
