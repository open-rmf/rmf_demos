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
import math
import yaml
import json
import time
import copy
import argparse

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default

from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy as History
from rclpy.qos import QoSDurabilityPolicy as Durability
from rclpy.qos import QoSReliabilityPolicy as Reliability

from rmf_fleet_msgs.msg import RobotState, Location, PathRequest, \
    DockSummary

import rmf_adapter as adpt
import rmf_adapter.vehicletraits as traits
import rmf_adapter.geometry as geometry

import numpy as np
from pyproj import Transformer

import socketio

from fastapi import FastAPI
import uvicorn
from typing import Optional
from pydantic import BaseModel

import threading
app = FastAPI()


class Request(BaseModel):
    map_name: str
    task: Optional[str] = None
    destination: Optional[dict] = None
    data: Optional[dict] = None
    speed_limit: Optional[float] = None


class Response(BaseModel):
    data: Optional[dict] = None
    success: bool
    msg: str


# ------------------------------------------------------------------------------
# Fleet Manager
# ------------------------------------------------------------------------------
class State:
    def __init__(self, state: RobotState = None, destination: Location = None):
        self.state = state
        self.destination = destination
        self.svy_transformer = Transformer.from_crs('EPSG:4326', 'EPSG:3414')
        self.gps_pos = [0, 0]

    def gps_to_xy(self, gps_json: dict):
        svy21_xy = \
            self.svy_transformer.transform(gps_json['lat'], gps_json['lon'])
        self.gps_pos[0] = svy21_xy[1]
        self.gps_pos[1] = svy21_xy[0]


class FleetManager(Node):
    def __init__(self, config, nav_path):
        self.config = config
        self.fleet_name = self.config["rmf_fleet"]["name"]

        self.gps = False
        self.offset = [0, 0]
        if 'reference_coordinates' in self.config and \
                'offset' in self.config['reference_coordinates']:
            assert len(self.config['reference_coordinates']['offset']) > 1, \
                ('Please ensure that the offset provided is valid.')
            self.gps = True
            self.offset = self.config['reference_coordinates']['offset']

        super().__init__(f'{self.fleet_name}_fleet_manager')

        self.robots = {}  # Map robot name to state
        self.docks = {}  # Map dock name to waypoints

        for robot_name, robot_config in self.config["robots"].items():
            self.robots[robot_name] = State()
        assert(len(self.robots) > 0)

        profile = traits.Profile(geometry.make_final_convex_circle(
            self.config['rmf_fleet']['profile']['footprint']),
            geometry.make_final_convex_circle(
                self.config['rmf_fleet']['profile']['vicinity']))
        self.vehicle_traits = traits.VehicleTraits(
            linear=traits.Limits(
                *self.config['rmf_fleet']['limits']['linear']),
            angular=traits.Limits(
                *self.config['rmf_fleet']['limits']['angular']),
            profile=profile)
        self.vehicle_traits.differential.reversible =\
            self.config['rmf_fleet']['reversible']

        self.sio = socketio.Client()

        @self.sio.on("/gps")
        def message(data):
            try:
                robot = json.loads(data)
                robot_name = robot['robot_id']
                self.robots[robot_name].gps_to_xy(robot)
            except KeyError as e:
                self.get_logger().info(f"Malformed GPS Message!: {e}")

        if self.gps:
            while True:
                try:
                    self.sio.connect('http://0.0.0.0:8080')
                    break
                except Exception:
                    self.get_logger().info(
                        f"Trying to connect to sio server at"
                        f"http://0.0.0.0:8080..")
                    time.sleep(1)

        self.create_subscription(
            RobotState,
            'robot_state',
            self.robot_state_cb,
            100)

        transient_qos = QoSProfile(
            history=History.KEEP_LAST,
            depth=1,
            reliability=Reliability.RELIABLE,
            durability=Durability.TRANSIENT_LOCAL)

        self.create_subscription(
            DockSummary,
            'dock_summary',
            self.dock_summary_cb,
            qos_profile=transient_qos)

        self.path_pub = self.create_publisher(
            PathRequest,
            'robot_path_requests',
            qos_profile=qos_profile_system_default)

        self.task_id = -1

        @app.get('/open-rmf/rmf_demos_fm/status/',
                 response_model=Response)
        async def position(robot_name: Optional[str] = None):
            data = {'data': {},
                    'success': False,
                    'msg': ''}
            if robot_name is None:
                data['data']['all_robots'] = []
                for robot_name in self.robots:
                    state = self.robots.get(robot_name)
                    if state is None or state.state is None:
                        return data
                    data['data']['all_robots'].append(
                        self.get_robot_state(state, robot_name))
            else:
                state = self.robots.get(robot_name)
                if state is None or state.state is None:
                    return data
                data['data'] = self.get_robot_state(state, robot_name)
            data['success'] = True
            return data

        @app.post('/open-rmf/rmf_demos_fm/navigate/',
                  response_model=Response)
        async def navigate(robot_name: str, dest: Request):
            data = {'success': False, 'msg': ''}
            if (robot_name not in self.robots or len(dest.destination) < 1):
                return data

            target_x = dest.destination['x']
            target_y = dest.destination['y']
            target_yaw = dest.destination['yaw']
            target_map = dest.map_name
            target_speed_limit = dest.speed_limit

            target_x -= self.offset[0]
            target_y -= self.offset[1]

            t = self.get_clock().now().to_msg()

            path_request = PathRequest()
            state = self.robots[robot_name]
            cur_x = state.state.location.x
            cur_y = state.state.location.y
            cur_yaw = state.state.location.yaw
            cur_loc = state.state.location
            path_request.path.append(cur_loc)

            disp = self.disp([target_x, target_y], [cur_x, cur_y])
            duration = int(disp/self.vehicle_traits.linear.nominal_velocity) +\
                int(abs(abs(cur_yaw) - abs(target_yaw)) /
                    self.vehicle_traits.rotational.nominal_velocity)
            t.sec = t.sec + duration
            target_loc = Location()
            target_loc.t = t
            target_loc.x = target_x
            target_loc.y = target_y
            target_loc.yaw = target_yaw
            target_loc.level_name = target_map
            if target_speed_limit > 0:
                target_loc.obey_approach_speed_limit = True
                target_loc.approach_speed_limit = target_speed_limit

            path_request.fleet_name = self.fleet_name
            path_request.robot_name = robot_name
            path_request.path.append(target_loc)
            self.task_id = self.task_id + 1
            path_request.task_id = str(self.task_id)
            self.path_pub.publish(path_request)

            self.robots[robot_name].destination = target_loc

            data['success'] = True
            return data

        @app.get('/open-rmf/rmf_demos_fm/stop_robot/',
                 response_model=Response)
        async def stop(robot_name: str):
            data = {'success': False, 'msg': ''}
            if robot_name not in self.robots:
                return data

            path_request = PathRequest()
            path_request.fleet_name = self.fleet_name
            path_request.robot_name = robot_name
            path_request.path = []
            self.task_id = self.task_id + 1
            path_request.task_id = str(self.task_id)
            self.path_pub.publish(path_request)
            data['success'] = True
            return data

        @app.post('/open-rmf/rmf_demos_fm/start_task/',
                  response_model=Response)
        async def start_process(robot_name: str, task: Request):
            data = {'success': False, 'msg': ''}
            if (robot_name not in self.robots or
                    len(task.task) < 1 or
                    task.task not in self.docks):
                return data

            path_request = PathRequest()
            state = self.robots[robot_name]
            cur_loc = state.state.location
            cur_x = cur_loc.x
            cur_y = cur_loc.y
            cur_yaw = cur_loc.yaw
            previous_wp = [cur_x, cur_y, cur_yaw]
            target_loc = Location()
            path_request.path.append(cur_loc)
            for wp in self.docks[task.task]:
                target_loc = wp
                path_request.path.append(target_loc)
                previous_wp = [wp.x, wp.y, wp.yaw]

            path_request.fleet_name = self.fleet_name
            path_request.robot_name = robot_name
            self.task_id = self.task_id + 1
            path_request.task_id = str(self.task_id)
            self.path_pub.publish(path_request)

            self.robots[robot_name].destination = target_loc

            data['success'] = True
            return data

    def robot_state_cb(self, msg):
        if (msg.name in self.robots):
            self.robots[msg.name].state = msg
            # Check if robot has reached destination
            state = self.robots[msg.name]
            if state.destination is None:
                return
            destination = state.destination
            if ((msg.mode.mode == 0 or msg.mode.mode == 1) and
                    len(msg.path) == 0):
                self.robots[msg.name].destination = None

    def dock_summary_cb(self, msg):
        for fleet in msg.docks:
            if(fleet.fleet_name == self.fleet_name):
                for dock in fleet.params:
                    self.docks[dock.start] = dock.path

    def get_robot_state(self, state, robot_name):
        data = {}
        if self.gps:
            position = copy.deepcopy(state.gps_pos)
        else:
            position = [state.state.location.x, state.state.location.y]
        angle = state.state.location.yaw
        data['robot_name'] = robot_name
        data['map_name'] = state.state.location.level_name
        data['position'] =\
            {'x': position[0], 'y': position[1], 'yaw': angle}
        data['battery'] = state.state.battery_percent
        data['completed_request'] = False
        if state.destination is not None:
            destination = state.destination
            # remove offset for calculation if using gps coords
            if self.gps:
                position[0] -= self.offset[0]
                position[1] -= self.offset[1]
            # calculate arrival estimate
            dist_to_target =\
                self.disp(position, [destination.x, destination.y])
            ori_delta = abs(abs(angle) - abs(destination.yaw))
            if ori_delta > np.pi:
                ori_delta = ori_delta - (2 * np.pi)
            if ori_delta < -np.pi:
                ori_delta = (2 * np.pi) + ori_delta
            duration = (dist_to_target /
                        self.vehicle_traits.linear.nominal_velocity +
                        ori_delta /
                        self.vehicle_traits.rotational.nominal_velocity)
            data['destination_arrival_duration'] = duration
        else:
            data['destination_arrival_duration'] = 0.0
            data['completed_request'] = True
        return data

    def disp(self, A, B):
        return math.sqrt((A[0]-B[0])**2 + (A[1]-B[1])**2)


# ------------------------------------------------------------------------------
# Main
# ------------------------------------------------------------------------------
def main(argv=sys.argv):
    # Init rclpy and adapter
    rclpy.init(args=argv)
    adpt.init_rclcpp()
    args_without_ros = rclpy.utilities.remove_ros_args(argv)

    parser = argparse.ArgumentParser(
        prog="fleet_adapter",
        description="Configure and spin up the fleet adapter")
    parser.add_argument("-c", "--config_file", type=str, required=True,
                        help="Path to the config.yaml file")
    parser.add_argument("-n", "--nav_graph", type=str, required=True,
                        help="Path to the nav_graph for this fleet adapter")
    args = parser.parse_args(args_without_ros[1:])
    print(f"Starting fleet manager...")

    with open(args.config_file, "r") as f:
        config = yaml.safe_load(f)

    fleet_manager = FleetManager(config, args.nav_graph)

    spin_thread = threading.Thread(target=rclpy.spin, args=(fleet_manager,))
    spin_thread.start()

    uvicorn.run(app,
                host=config['rmf_fleet']['fleet_manager']['ip'],
                port=config['rmf_fleet']['fleet_manager']['port'],
                log_level='warning')


if __name__ == '__main__':
    main(sys.argv)
