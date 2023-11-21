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
import math

import numpy as np

import rclpy

from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_system_default
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy as History
from rclpy.qos import QoSDurabilityPolicy as Durability
from rclpy.qos import QoSReliabilityPolicy as Reliability
from rclpy.time import Time
from rclpy.duration import Duration

from rmf_task_msgs.msg import ApiRequest, ApiResponse
from rmf_chope_msgs.srv import FixedTimeRequest, ClaimReservation
from rmf_chope_msgs.msg import FixedTimeReservationAlt
from rmf_fleet_msgs.msg import RobotState
from rmf_building_map_msgs.msg import Graph

from enum import Enum

###############################################################################

class AgentState(Enum):
    ACTIVE = 1
    REQUESTED_CHARGE = 2
    CLAIMED_CHARGE = 3
    PROCEED_TO_CHARGE = 4
    CHARGING = 5
    PARKED = 6
    SWAP_PARKING = 7

class ParkingState(Enum):
    RESERVERED = 0
    NEXT_BID = 1

parking_spots = {"pantry", "lounge", "supplies"}
chargers = {"pantry"}

CHARGE_DURATION = 100
CLOCK_RATE = 0.1
OPERATION_DURATION = 60
PARKING_SPOT_HOLD_DUR = 50
THRESHOLD = 100 * (1-PARKING_SPOT_HOLD_DUR / CHARGE_DURATION)

class ChargerDance(Node):

    def __init__(self, argv=sys.argv):
        super().__init__('charger_dance')
        parser = argparse.ArgumentParser()
        parser.add_argument('-F', '--fleet', type=str, help='Fleet name')
        parser.add_argument('-R', '--robot', type=str, help='Robot name')
        parser.add_argument("--use-sim-time", action="store_true",
                            help='Use sim time, default: false')

        self.args = parser.parse_args(argv[1:])
        # enable ros sim time
        if self.args.use_sim_time:
            self.get_logger().info("Using Sim Time")
            param = Parameter("use_sim_time", Parameter.Type.BOOL, True)
            self.set_parameters([param])

        self.reservation_request_client = \
            self.create_client(FixedTimeRequest, '/rmf/reservation_node/request')
        self.claim_client = \
            self.create_client(ClaimReservation, '/rmf/reservation_node/claim')
        self.ticket = None

        self.state = AgentState.ACTIVE

        self.current_battery = 100

        latching_qos = QoSProfile(depth=1,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL)

        self.sub = self.create_subscription(
            ApiResponse, 'task_api_responses', self.receive_response, 10
        )

        self.robot_state_sub = self.create_subscription(
            RobotState, 'robot_state', self.robot_state_update, 10
        )

        self.nav_graph_sub = self.create_subscription(
            Graph, 'nav_graphs', self.get_nav_graph, latching_qos
        )

        transient_qos = QoSProfile(
            history=History.KEEP_LAST,
            depth=1,
            reliability=Reliability.RELIABLE,
            durability=Durability.TRANSIENT_LOCAL)

        self.pub = self.create_publisher(
          ApiRequest, 'task_api_requests', transient_qos
        )

        self.timer = self.create_timer(OPERATION_DURATION*0.3, self.request_charger)

        self.nav_graph = None

        self.distance_to_location = {}

        self.robot_location = None

        self.prev_location = None

        self.parking_state = ParkingState.NEXT_BID

        self.last_reservation = None

    def request_charger(self):
        self.timer.cancel()
        if len(self.distance_to_location) == 0:
            # Distance is not yet ready
            print("Delaying start as we have not yet localized ourselves")
            self.create_timer(0.1, self.request_charger)
            return
        self.next_reservation_start = self.get_clock().now() + Duration(seconds = OPERATION_DURATION)
        self.generate_alternatives(chargers, self.next_reservation_start.to_msg(), CHARGE_DURATION)
        self.next_reservation_start += Duration(seconds=CHARGE_DURATION)
        self.request_resource()
        self.timer = self.create_timer(
            OPERATION_DURATION * 0.7,
            self.claim_charger)

    def claim_charger(self):
        self.timer.cancel()
        self.claim_and_rush_to_spot()
        self.timer = self.create_timer(
            OPERATION_DURATION * 0.3,
            self.check_reached_charger)
 

    def check_reached_charger(self):
        print("Checking if reached charger")
        self.timer.cancel()
        if self.robot_location in chargers:
            self.timer = self.create_timer(0.1, self.check_reached_charger)
            # TODO(arjo) Try to extend charger claim
            return
        
        self.generate_alternatives(chargers, self.next_reservation_start.to_msg(), OPERATION_DURATION)
        self.timer = self.create_timer(
            Duration(seconds=self.get_clock().now() + self.reservation_end),
            self.switch_to_parked_mode)
        
    def switch_to_parked_mode(self):
        pass
        
    def get_nav_graph(self, building_msgs: Graph):
        print("Got graph")
        self.nav_graph = building_msgs

    def robot_state_update(self, robot_state_msgs: RobotState):
        #print("Got update")
        if self.args.robot == robot_state_msgs.name and self.nav_graph is not None:
            x_loc = robot_state_msgs.location.x
            y_loc = robot_state_msgs.location.y

            location = None
            for node in self.nav_graph.vertices:
                dist = (x_loc - node.x) ** 2 + (y_loc - node.y) ** 2
                self.distance_to_location[node.name] = dist
                if dist < 0.001:
                    location = node.name
            self.robot_location = location

    def generate_alternatives(self, parameters, start_time, duration):
        self.alternatives = []
        for alt in parameters:
            alternative = FixedTimeReservationAlt()
            alternative.resource_name = alt
            #Dumb heuristic. Nearest spot (euclidean).
            alternative.cost = self.distance_to_location[alt]
            alternative.start_time = start_time
            alternative.has_end = True
            alternative.duration = Duration(seconds=duration).to_msg()
            self.alternatives.append(alternative)
        print(f"Generated {len(self.alternatives)} alternatives")

    def parking_manager(self):
        if self.state == AgentState.PARKED:
            if self.parking_state == ParkingState.NEXT_BID:
                claim_time = self.get_clock().now() + Duration(seconds=PARKING_SPOT_HOLD_DUR)
                self.generate_alternatives(parking_spots, claim_time.to_msg(), PARKING_SPOT_HOLD_DUR)
                request = FixedTimeRequest.Request()
                request.alternatives = self.alternatives
                fut = self.reservation_request_client.call_async(request)
                fut.add_done_callback(self.on_get_assigned_ticket)
                self.parking_state = ParkingState.RESERVERED
            else:
                print("Claiming charger spot")
                self.claim_and_rush_to_spot()
                self.parking_state = ParkingState.NEXT_BID

    def request_resource(self):
        request = FixedTimeRequest.Request()
        request.alternatives = self.alternatives
        fut = self.reservation_request_client.call_async(request)
        fut.add_done_callback(self.on_get_assigned_ticket)

    def receive_response(self, response_msg: ApiResponse):
        pass
        #if response_msg.request_id == msg.request_id:
        #    self.response.set_result(json.loads(response_msg.json_msg))
        #
        #

    def on_arrive_at_location(self):
        # See if last reservation is long enough
        if self.prev_location != self.robot_location:
            pass       

    def move_to_location_based_on_rsys(self, x):
        if x.result().ok:
            print ("Selecting: ")
            move_idx = x.result().alternative
            print("Moving to " + self.alternatives[move_idx].resource_name)
            if self.alternatives[move_idx].has_end:
                self.reservation_end = self.alternatives[move_idx].start_time + self.alternatives[move_idx].end_time
            else:
                self.reservation_end = None
            self.goto_place(self.alternatives[move_idx].resource_name) 
        else:
            print("Could not claim spot! Panic! Give UP!")

    def claim_and_rush_to_spot(self):
        req = ClaimReservation.Request()
        req.ticket = self.ticket
        fut = self.claim_client.call_async(req)
        fut.add_done_callback(self.move_to_location_based_on_rsys)

    def claim_and_rush_to_charger(self):
        if self.state == AgentState.REQUESTED_CHARGE and self.ticket is not None:
            print("Claiming spot")
            req = ClaimReservation.Request()
            req.ticket = self.ticket
            fut = self.claim_client.call_async(req)
            fut.add_done_callback(self.move_to_location_based_on_rsys)
            self.state = AgentState.CHARGING

    def on_get_assigned_ticket(self, x):
        if x.result().ok:
            print("Got ticket")
            self.ticket = x.result().ticket
        else:
            print("Could not get ticket")

    def on_battery_warning(self):
        if self.state == AgentState.ACTIVE:
            print("Requesting charger")
            self.state = AgentState.REQUESTED_CHARGE
            self.last_reservation = (self.get_estimated_use_time(), CHARGE_DURATION)
            self.generate_alternatives(chargers, self.get_estimated_use_time().to_msg(), CHARGE_DURATION)
            request = FixedTimeRequest.Request()
            request.alternatives = self.alternatives
            fut = self.reservation_request_client.call_async(request)
            fut.add_done_callback(self.on_get_assigned_ticket)

    def goto_place(self, place):
        msg = ApiRequest()
        msg.request_id = "direct_" + str(uuid.uuid4())
        payload = {}

        #if self.args.robot and self.args.fleet:
        self.get_logger().info("Using 'robot_task_request'")
        payload["type"] = "robot_task_request"
        payload["robot"] = self.args.robot
        payload["fleet"] = "tinyRobot"

        # Set task request start time
        now = self.get_clock().now().to_msg()
        now.sec = now.sec
        start_time = now.sec * 1000 + round(now.nanosec/10**6)

        # Define task request description
        go_to_description = {'waypoint': place}

        go_to_activity = {
            'category': 'go_to_place',
            'description': go_to_description
        }

        rmf_task_request = {
            'category': 'compose',
            'description': {
                'category': 'go_to_place',
                'phases': [{'activity': go_to_activity}]
            },
            'unix_millis_earliest_start_time': start_time
        }

        payload["request"] = rmf_task_request

        msg.json_msg = json.dumps(payload)

        print(f"Json msg payload: \n{json.dumps(payload, indent=2)}")

        self.pub.publish(msg)


###############################################################################


def main(argv=sys.argv):
    rclpy.init(args=sys.argv)
    args_without_ros = rclpy.utilities.remove_ros_args(sys.argv)

    charger_dance = ChargerDance(args_without_ros)
    #executor = MultiThreadedExecutor(num_threads=8)
    rclpy.spin(charger_dance)#, executor=executor)
    rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv)
