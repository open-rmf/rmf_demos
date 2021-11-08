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

#! /usr/bin/env python3
import sys
import math
import yaml
import argparse

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from rclpy.parameter import Parameter

from rmf_fleet_msgs.msg import RobotState, FleetState, Location, PathRequest

import rmf_adapter as adpt
import rmf_adapter.vehicletraits as traits
import rmf_adapter.geometry as geometry
import rmf_adapter.graph as graph
import rmf_adapter.plan as plan

import numpy as np

from fastapi import FastAPI
import uvicorn
import contextlib
import time
from typing import Optional, Dict
from pydantic import BaseModel

import threading
app = FastAPI()

class Server(uvicorn.Server):
    def install_signal_handlers(self):
        pass

    @contextlib.contextmanager
    def run_in_thread(self):
        thread = threading.Thread(target=self.run)
        thread.start()
        try:
            while not self.started:
                time.sleep(1e-3)
            yield
        finally:
            self.should_exit = True
            thread.join()

class Request(BaseModel):
    map_name: str
    task: Optional[str] = None
    destination: Optional[dict] = None
    data: Optional[dict] = None

# ------------------------------------------------------------------------------
# Fleet Manager
# ------------------------------------------------------------------------------
class FleetManager(Node):
    def __init__(self, config_path, nav_path, port):
        super().__init__('rmf_demos_fleet_manager')
        self.port = port

        self.config = None
        with open(config_path, "r") as f:
            self.config = yaml.safe_load(f)
        self.fleet_name = self.config["rmf_fleet"]["name"]
        self.robot_names = []
        self.prefix = ''

        for robot_name, robot_config in self.config["robots"].items():
            self.robot_names.append(robot_name)
            self.map_name = robot_config["rmf_config"]["start"]["map_name"]
            self.prefix = robot_config['robot_config']['base_url']
        assert(len(self.robot_names) > 0)

        profile = traits.Profile(geometry.make_final_convex_circle(
            self.config['rmf_fleet']['profile']['footprint']),
            geometry.make_final_convex_circle(self.config['rmf_fleet']['profile']['vicinity']))
        self.vehicle_traits = traits.VehicleTraits(
            linear=traits.Limits(*self.config['rmf_fleet']['limits']['linear']),
            angular=traits.Limits(*self.config['rmf_fleet']['limits']['angular']),
            profile=profile)
        self.vehicle_traits.differential.reversible = self.config['rmf_fleet']['reversible']
        # nav_graph = graph.parse_graph(nav_path, self.vehicle_traits)
        # config = plan.Configuration(nav_graph, self.vehicle_traits)
        # self.planner = plan.Planner(config)

        self.create_subscription(
            RobotState,
            'robot_state',
            self.robot_state_cb,
            10)

        self.path_pub = self.create_publisher(
            PathRequest,
            'robot_path_requests',
            qos_profile=qos_profile_system_default)

        self.state = {}
        self.destination = {} # stores destination waypoints and time for each robot: {"position":[x,y,yaw],"t":Time}}
        self.task_id = -1

        @app.get('/open-rmf/rmf_demos_fm/status/')
        async def position(robot_name: Optional[str] = None):
            data = {'data': {'robot_name':robot_name, 
                             'map_name':'',
                             'position':{'x':0.0,'y':0.0,'yaw':0.0},
                             'battery':0.0,
                             'destination':{'x':0.0,'y':0.0,'yaw':0.0},
                             'destination_arrival_duration':0.0,
                             'completed_request':False},
                    'success':False,
                    'msg':''}
            if robot_name not in self.state:
                return data
            position = [self.state[robot_name].location.x, self.state[robot_name].location.y]
            angle = self.state[robot_name].location.yaw
            data['data']['robot_name'] = robot_name
            data['data']['map_name'] = self.map_name
            data['data']['position'] = {'x':position[0],'y':position[1],'yaw':angle}
            data['data']['battery'] = self.state[robot_name].battery_percent
            if self.destination[robot_name] is not None:
                destination = self.destination[robot_name]
                data['data']['destination'] = {'x':destination.x,'y':destination.y,'yaw':destination.yaw}
                data['data']['destination_arrival_duration'] = destination.t.sec - self.state[robot_name].location.t.sec
                if ((self.state[robot_name].mode.mode == 0 or self.state[robot_name].mode.mode == 1)) and\
                    ((abs(position[0]-data['data']['destination']['x']) < 0.5) and\
                    (abs(position[1]-data['data']['destination']['y']) < 0.5) and\
                    (abs(angle-data['data']['destination']['yaw']) < 0.1)):
                    data['data']['completed_request'] = True
                    self.destination[robot_name] = None
            data['success'] = True
            return data

        @app.post('/open-rmf/rmf_demos_fm/navigate/')
        async def navigate(robot_name: str, dest: Request):
            data = {'success': False, 'msg': ''}
            if (robot_name not in self.robot_names or\
                dest.map_name != self.map_name or\
                len(dest.destination) < 1):
                return data

            target_x = dest.destination['x']
            target_y = dest.destination['y']
            target_yaw = dest.destination['yaw']

            # # Add some noise to the actual location the robot will navigate to
            # target_x = target_x + np.random.uniform(-0.5, 0.5)
            # target_y = target_y + np.random.uniform(-0.5, 0.5)

            t = self.get_clock().now().to_msg()

            path_request = PathRequest()
            cur_x = self.state[robot_name].location.x
            cur_y = self.state[robot_name].location.y
            cur_yaw = self.state[robot_name].location.yaw
            cur_loc = self.state[robot_name].location
            path_request.path.append(cur_loc)

            disp = self.disp([target_x, target_y], [cur_x, cur_y])
            duration = int(disp/self.vehicle_traits.linear.nominal_velocity)
            t.sec = t.sec + duration
            target_loc = Location()
            target_loc.t = t
            target_loc.x = target_x
            target_loc.y = target_y
            target_loc.yaw = target_yaw
            target_loc.level_name = self.map_name
            
            path_request.fleet_name = self.fleet_name
            path_request.robot_name = robot_name
            path_request.path.append(target_loc)
            self.task_id = self.task_id + 1
            path_request.task_id = str(self.task_id)
            self.path_pub.publish(path_request)

            self.destination[robot_name] = target_loc

            data['success'] = True
            return data

        @app.get('/open-rmf/rmf_demos_fm/stop_robot/')
        async def stop(robot_name: str):
            data = {'success': False, 'msg': ''}
            if robot_name not in self.robot_names:
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

        @app.post('/open-rmf/rmf_demos_fm/start_task/')
        async def start_process(robot_name: str, task: Request):
            data = {'success': False, 'msg': ''}
            # print(f"Request data: {task}")
            if (robot_name not in self.robot_names or\
                task.map_name != self.map_name or\
                len(task.task) < 1):
                return data
            # ------------------------ #
            # TODO START PROCESS HERE
            # ------------------------ #
            data['success'] = True
            return data

    def robot_state_cb(self, msg):
        if (msg.name in self.robot_names):
            self.state[msg.name] = msg

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
    parser.add_argument('-p', '--port', help='Port for API Server')
    args = parser.parse_args(args_without_ros[1:])
    print(f"Starting fleet manager...")

    fleet_manager = FleetManager(args.config_file, args.nav_graph, args.port)

    spin_thread = threading.Thread(target=rclpy.spin, args=(fleet_manager,))
    spin_thread.start()

    uvicorn.run(app, host='127.0.0.1', port=8001, log_level='warning')

if __name__ == '__main__':
    main(sys.argv)