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


import nudged
import socketio
import copy
import json
import math
import time
import numpy as np
from pyproj import Transformer
from rmf_fleet_msgs.msg import RobotState, Location, PathRequest, \
    ModeRequest, ModeParameter
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from shapely.geometry import Polygon, Point


class State:
    def __init__(self, vehicle_traits):
        self.initialized = False

        # To be initialized over gps update
        self.current_loc = Location()
        # To be updated by adapter tasks
        self.target_loc = Location()

        self.vehicle_traits = vehicle_traits

    def disp(self):
        return self._disp(self.target_loc, self.current_loc)

    def duration_to_target(self):
        return int(self.disp()/self.vehicle_traits.linear.nominal_velocity)

    def _disp(self, A: Location, B: Location):
        return math.sqrt((A.x-B.x)**2 + (A.y-B.y)**2)

    def update_current_loc_from_gps_message(self, gps_json: dict):
        raise NotImplementedError


class RobotAPI(Node):
    def __init__(
            self, prefix: str, user: str, password: str,
            robot_name: str, config: dict, vehicle_traits: dict):
        super().__init__(f"{robot_name}_robot_api")
        # API connection to robot
        self.prefix = prefix
        self.user = user
        self.password = password
        self.connected = False
        self.robot_name = robot_name
        self.config = config
        self.fleet_name = self.config["fleet_name"]
        self.sio = socketio.Client()

        @self.sio.on("/gps")
        def message(data):
            raise NotImplementedError

        self.state = State(vehicle_traits)

        # Task housekeeping
        self.task_id = -1

        # GPS Transformation

        self._init_pubsub()

        # Test connectivity
        self.sio.connect(self.prefix)
        connected = self.check_connection()
        if connected:
            self.connected = True
        else:
            print("Unable to query GPS server")

    def _init_pubsub(self):
        self.path_pub = self.create_publisher(
            PathRequest,
            'robot_path_requests',
            qos_profile=qos_profile_system_default)

        self.mode_pub = self.create_publisher(
            ModeRequest,
            'robot_mode_requests',
            qos_profile=qos_profile_system_default)

    def check_connection(self):
        ''' Return True if connection to the robot API server is successful'''
        raise NotImplementedError

    def position(self):
        raise NotImplementedError

    def navigate(self, pose, map_name: str):
        ''' Request the robot to navigate to pose:[x,y,theta] where x, y and
            and theta are in the robot's coordinate convention. This function
            should return True if the robot has accepted the request,
            else False'''

        # Compute displacement required
        # BH(TODO): Error handling
        self.state.target_loc = Location()
        self.state.target_loc.x = pose[0]
        self.state.target_loc.y = pose[1]
        self.state.target_loc.yaw = pose[2]

        t = self.get_clock().now().to_msg()
        duration_to_target_loc = self.state.duration_to_target()
        if duration_to_target_loc is None:
            return False

        t.sec = t.sec + duration_to_target_loc
        path_request = PathRequest()

        path_request.path.append(self.state.current_loc)
        path_request.fleet_name = self.fleet_name
        path_request.robot_name = self.robot_name
        path_request.path.append(self.state.target_loc)

        self.task_id += 1
        path_request.task_id = str(self.task_id)
        self.path_pub.publish(path_request)

        return True

    def start_process(self, process: str, map_name: str):
        return False

    def stop(self):
        path_request = PathRequest()
        path_request.fleet_name = self.fleet_name
        path_request.robot_name = self.robot_name
        path_request.path = []
        self.task_id = self.task_id + 1
        path_request.task_id = str(self.task_id)
        self.path_pub.publish(path_request)
        return True

    def navigation_remaining_duration(self):
        ''' Return the number of seconds remaining for the robot to reach its
            destination'''
        return self.state.duration_to_target()

    def navigation_completed(self):
        ''' Return True if the robot has successfully completed its previous
            navigation request. Else False.'''
        duration = self.state.duration_to_target()

        # BH(WARN): Arbitrary threshold
        if (duration < 0.1):
            return True
        else:
            return False

    def process_completed(self):
        return False

    def battery_soc(self):
        # BH(TODO): Ain't got time for that, full charge!
        return 1.0
