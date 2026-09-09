#!/usr/bin/env python3

# Copyright 2026 Open Source Robotics Foundation, Inc.
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

"""
Fleet manager for the Path Guide demo adapter.

Based on rmf_demos_fleet_adapter/fleet_manager.py. The per-waypoint navigate
endpoint is replaced by POST /follow-path, which receives the robot's entire
route in one request and publishes it as a single multi-point PathRequest. The
routes live under the /open-rmf/rmf_demos_pg/ prefix so this manager can be
told apart from the EasyFullControl one in logs and curl.
"""

import argparse
import copy
import json
import math
import sys
import threading
import time
from typing import Optional

from fastapi import FastAPI
import numpy as np
from pydantic import BaseModel
from pyproj import Transformer
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from rclpy.qos import QoSDurabilityPolicy as Durability
from rclpy.qos import QoSHistoryPolicy as History
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy as Reliability
import rmf_adapter as adpt
import rmf_adapter.geometry as geometry
import rmf_adapter.vehicletraits as traits
from rmf_fleet_msgs.msg import DockSummary
from rmf_fleet_msgs.msg import Location
from rmf_fleet_msgs.msg import ModeRequest
from rmf_fleet_msgs.msg import PathRequest
from rmf_fleet_msgs.msg import RobotMode
from rmf_fleet_msgs.msg import RobotState
import socketio
import uvicorn
import yaml

app = FastAPI()

# Fallback tolerance for a waypoint that arrives without an arrival_radius.
# The adapter always supplies one per waypoint, so this only covers callers
# hitting POST /follow-path directly.
DEFAULT_ARRIVAL_RADIUS = 0.3


class Request(BaseModel):
    map_name: Optional[str] = None
    activity: Optional[str] = None
    label: Optional[str] = None
    destination: Optional[dict] = None
    waypoints: Optional[list] = None
    data: Optional[dict] = None
    speed_limit: Optional[float] = None
    toggle: Optional[bool] = None


class Response(BaseModel):
    data: Optional[dict] = None
    success: bool
    msg: str


# -----------------------------------------------------------------------------
# Fleet Manager
# -----------------------------------------------------------------------------
class State:

    def __init__(self, state: RobotState = None, destination: Location = None):
        self.state = state
        self.destination = destination
        self.last_path_request = None
        self.last_completed_request = None
        # The current path as this manager measures arrival against it: one
        # entry per waypoint, in the frame the robot is commanded in, carrying
        # the arrival_radius the adapter chose for it. Kept alongside
        # last_path_request because rmf_fleet_msgs/Location has nowhere to put
        # a tolerance.
        # Guards the three fields below, which are written by POST
        # /follow-path on the uvicorn thread and by advance_reached_index on
        # the rclpy spin thread. Without it a spin thread preempted between
        # computing `furthest` and assigning it writes the OLD path's index
        # back after follow_path reset it -- so /status serves the new path_id
        # with a stale index, and the adapter credits waypoints the robot never
        # visited. Nothing blocking runs under it.
        self.progress_lock = threading.Lock()
        self.path_waypoints = []
        self.path_id = None
        # Highest waypoint index the robot has been seen to reach, -1 before
        # any. Only ever increases within a path.
        self.reached_index = -1
        self.perform_action_mode = False
        self.svy_transformer = Transformer.from_crs('EPSG:4326', 'EPSG:3414')
        self.gps_pos = [0, 0]

    def gps_to_xy(self, gps_json: dict):
        svy21_xy = self.svy_transformer.transform(
            gps_json['lat'], gps_json['lon']
        )
        self.gps_pos[0] = svy21_xy[1]
        self.gps_pos[1] = svy21_xy[0]

    def is_expected_task_id(self, task_id):
        if self.last_path_request is not None:
            if task_id != self.last_path_request.task_id:
                return False
        return True


class FleetManager(Node):

    def __init__(self, config):
        self.debug = False
        self.config = config
        self.fleet_name = self.config['rmf_fleet']['name']
        mgr_config = self.config['fleet_manager']
        self.ignore_speed_limit = mgr_config.get('ignore_speed_limit', False)

        self.gps = False
        self.offset = [0, 0]
        reference_coordinates_yaml = mgr_config.get('reference_coordinates')
        if reference_coordinates_yaml is not None:
            offset_yaml = reference_coordinates_yaml.get('offset')
            if offset_yaml is not None and len(offset_yaml) > 1:
                self.gps = True
                self.offset = offset_yaml

        super().__init__(f'{self.fleet_name}_fleet_manager')

        self.robots = {}  # Map robot name to state
        self.action_paths = {}  # Map activities to paths
        self.docks = {}  # Map dock start name to its path

        for robot_name, _ in self.config['rmf_fleet']['robots'].items():
            self.robots[robot_name] = State()
        assert len(self.robots) > 0

        profile = traits.Profile(
            geometry.make_final_convex_circle(
                self.config['rmf_fleet']['profile']['footprint']
            ),
            geometry.make_final_convex_circle(
                self.config['rmf_fleet']['profile']['vicinity']
            ),
        )
        self.vehicle_traits = traits.VehicleTraits(
            linear=traits.Limits(
                *self.config['rmf_fleet']['limits']['linear']
            ),
            angular=traits.Limits(
                *self.config['rmf_fleet']['limits']['angular']
            ),
            profile=profile,
        )
        self.vehicle_traits.differential.reversible = self.config['rmf_fleet'][
            'reversible'
        ]

        fleet_manager_config = self.config['fleet_manager']
        self.action_paths = fleet_manager_config.get('action_paths', {})
        self.sio = socketio.Client()

        @self.sio.on('/gps')
        def message(data):
            try:
                robot = json.loads(data)
                robot_name = robot['robot_id']
                self.robots[robot_name].gps_to_xy(robot)
            except KeyError as e:
                self.get_logger().info(f'Malformed GPS Message!: {e}')

        if self.gps:
            while True:
                try:
                    self.sio.connect('http://0.0.0.0:8080')
                    break
                except Exception:
                    self.get_logger().info(
                        'Trying to connect to sio server at '
                        'http://0.0.0.0:8080..'
                    )
                    time.sleep(1)

        self.create_subscription(
            RobotState, 'robot_state', self.robot_state_cb, 100
        )

        transient_qos = QoSProfile(
            history=History.KEEP_LAST,
            depth=1,
            reliability=Reliability.RELIABLE,
            durability=Durability.TRANSIENT_LOCAL,
        )

        self.create_subscription(
            DockSummary,
            'dock_summary',
            self.dock_summary_cb,
            qos_profile=transient_qos,
        )

        publisher_qos = QoSProfile(
            history=History.KEEP_LAST,
            depth=10,
            reliability=Reliability.RELIABLE,
            durability=Durability.VOLATILE)

        self.path_pub = self.create_publisher(
            PathRequest,
            'robot_path_requests',
            qos_profile=qos_profile_system_default,
        )

        self.mode_pub = self.create_publisher(
            ModeRequest,
            'robot_mode_requests',
            qos_profile=publisher_qos)

        self.action_completed_pub = self.create_publisher(
            ModeRequest,
            'action_execution_notice',
            qos_profile=qos_profile_system_default,
        )

        @app.get('/open-rmf/rmf_demos_pg/status/', response_model=Response)
        async def status(robot_name: Optional[str] = None):
            response = {'data': {}, 'success': False, 'msg': ''}
            if robot_name is None:
                response['data']['all_robots'] = []
                for robot_name in self.robots:
                    state = self.robots.get(robot_name)
                    if state is None or state.state is None:
                        return response
                    response['data']['all_robots'].append(
                        self.get_robot_state(state, robot_name)
                    )
            else:
                state = self.robots.get(robot_name)
                if state is None or state.state is None:
                    return response
                response['data'] = self.get_robot_state(state, robot_name)
            response['success'] = True
            return response

        @app.post(
            '/open-rmf/rmf_demos_pg/follow-path/', response_model=Response
        )
        async def follow_path(robot_name: str, path_id: int, req: Request):
            """
            Send the robot its entire route in one PathRequest.

            This is the whole point of the Path Guide adapter: where the
            EasyFullControl demo manager receives one destination at a time and
            publishes a two-point PathRequest, this receives every waypoint up
            front and publishes them all, so the robot can plan across the
            route rather than vertex to vertex.
            """
            response = {'success': False, 'msg': ''}
            if robot_name not in self.robots:
                response['msg'] = f'Unknown robot [{robot_name}]'
                return response

            if not req.waypoints:
                response['msg'] = 'No waypoints were provided'
                return response

            robot = self.robots[robot_name]
            if robot.state is None:
                response['msg'] = f'No state yet for robot [{robot_name}]'
                return response

            path_request = PathRequest()
            path_request.fleet_name = self.fleet_name
            path_request.robot_name = robot_name
            path_request.task_id = str(path_id)

            # Start from wherever the robot is now, then lay out the whole
            # route. Each leg's arrival time accumulates so the slotcar plugin
            # gets a monotonically increasing schedule.
            cur_loc = robot.state.location
            path_request.path.append(cur_loc)

            t = self.get_clock().now().to_msg()
            prev_x = cur_loc.x
            prev_y = cur_loc.y
            prev_yaw = cur_loc.yaw
            target_loc = None
            measured_waypoints = []

            for wp in req.waypoints:
                target_x = wp['x'] - self.offset[0]
                target_y = wp['y'] - self.offset[1]
                target_yaw = wp['yaw']
                target_map = wp['map_name']

                speed_limit = wp.get('speed_limit')
                if self.ignore_speed_limit:
                    speed_limit = None

                # NOTE: wp['arrival_radius'] cannot be passed to the robot --
                # rmf_fleet_msgs/Location has nowhere to carry a tolerance --
                # so it is kept alongside the PathRequest in
                # `measured_waypoints` below and used by advance_reached_index
                # to decide when each waypoint has been reached. That decision
                # belongs on this side; the adapter measures nothing.
                #
                # It arrives already capped for lift cabins. Do not second-guess
                # it here: the adapter is the only side that sees
                # `destination.inside_lift`, and dropping the value it sent
                # reinstates the doors-closing-on-the-robot bug.

                duration = int(
                    self.disp([target_x, target_y], [prev_x, prev_y])
                    / self.vehicle_traits.linear.nominal_velocity
                ) + int(
                    abs(abs(prev_yaw) - abs(target_yaw))
                    / self.vehicle_traits.rotational.nominal_velocity
                )
                t = copy.deepcopy(t)
                t.sec = t.sec + duration

                target_loc = Location()
                target_loc.t = t
                target_loc.x = target_x
                target_loc.y = target_y
                target_loc.yaw = target_yaw
                target_loc.level_name = target_map
                target_loc.obey_approach_speed_limit = False
                if speed_limit is not None and speed_limit > 0.0:
                    target_loc.obey_approach_speed_limit = True
                    target_loc.approach_speed_limit = speed_limit

                path_request.path.append(target_loc)
                # Same coordinates the robot is commanded in, so arrival can be
                # measured against its reported position with no second
                # transform. \sa advance_reached_index
                measured_waypoints.append(
                    {
                        'x': target_x,
                        'y': target_y,
                        'map_name': target_map,
                        # `is None`, not `or`: a legitimate 0.0 is falsey, and
                        # `0.0 or 0.3` would WIDEN the radius -- the one thing
                        # the note above forbids doing to a value that arrives
                        # already capped for lift cabins.
                        'arrival_radius': float(
                            DEFAULT_ARRIVAL_RADIUS
                            if wp.get('arrival_radius') is None
                            else wp['arrival_radius']
                        ),
                    }
                )
                prev_x = target_x
                prev_y = target_y
                prev_yaw = target_yaw

            self.path_pub.publish(path_request)

            if self.debug:
                print(
                    f'Sending follow-path request for {robot_name}: '
                    f'{path_id} with {len(req.waypoints)} waypoints'
                )
            robot.last_path_request = path_request
            with robot.progress_lock:
                robot.path_waypoints = measured_waypoints
                robot.path_id = str(path_id)
                robot.reached_index = -1
            # The final waypoint is what the arrival estimate in
            # get_robot_state is measured against.
            robot.destination = target_loc

            response['success'] = True
            return response

        @app.get(
            '/open-rmf/rmf_demos_pg/stop-robot/', response_model=Response
        )
        async def stop(
            robot_name: str, running_cmd_id: int, stop_cmd_id: int
        ):
            response = {'success': False, 'msg': ''}
            if robot_name not in self.robots:
                return response

            robot = self.robots[robot_name]
            path_request = PathRequest()
            path_request.fleet_name = self.fleet_name
            path_request.robot_name = robot_name
            path_request.path = []
            # Appending the current location twice will effectively tell the
            # robot to stop
            path_request.path.append(robot.state.location)
            path_request.path.append(robot.state.location)

            path_request.task_id = str(stop_cmd_id)
            self.path_pub.publish(path_request)

            if self.debug:
                print(
                    f'Sending stop request for {robot_name}: {running_cmd_id}'
                )
            robot.last_path_request = path_request
            robot.destination = None

            response['success'] = True
            return response

        @app.get(
            '/open-rmf/rmf_demos_pg/action-paths/', response_model=Response
        )
        async def action_paths(activity: str, label: str):
            response = {'success': False, 'msg': ''}
            if activity not in self.action_paths:
                return response

            if label not in self.action_paths[activity]:
                return response

            response['data'] = self.action_paths[activity][label]
            response['success'] = True
            return response

        @app.post(
            '/open-rmf/rmf_demos_pg/start-activity/', response_model=Response
        )
        async def start_activity(
            robot_name: str, cmd_id: int, request: Request
        ):
            response = {'success': False, 'msg': ''}
            if (
                robot_name not in self.robots
                or request.activity not in self.action_paths
                or request.label not in self.action_paths[request.activity]
            ):
                return response

            robot = self.robots[robot_name]

            path_request = PathRequest()
            cur_loc = robot.state.location
            target_loc = Location()
            path_request.path.append(cur_loc)

            activity_path = self.action_paths[request.activity][request.label]
            map_name = activity_path['map_name']
            for wp in activity_path['path']:
                target_loc = Location()
                target_loc.x = wp[0]
                target_loc.y = wp[1]
                target_loc.yaw = wp[2]
                target_loc.level_name = map_name
                path_request.path.append(target_loc)

            path_request.fleet_name = self.fleet_name
            path_request.robot_name = robot_name
            path_request.task_id = str(cmd_id)
            self.path_pub.publish(path_request)

            if self.debug:
                print(
                    f'Sending [{request.activity}] at [{request.label}] '
                    f'request for {robot_name}: {cmd_id}'
                )
            robot.last_path_request = path_request
            robot.destination = target_loc

            response['success'] = True
            response['data'] = {}
            response['data']['path'] = activity_path
            return response

        @app.post(
            '/open-rmf/rmf_demos_pg/toggle-teleop/', response_model=Response
        )
        async def toggle_teleop(robot_name: str, mode: Request):
            response = {'success': False, 'msg': ''}
            if robot_name not in self.robots:
                return response
            # Toggle action mode
            self.robots[robot_name].perform_action_mode = mode.toggle
            response['success'] = True
            return response

        @app.post(
            '/open-rmf/rmf_demos_pg/toggle-attach/', response_model=Response
        )
        async def toggle_attach(robot_name: str, cmd_id: int, mode: Request):
            response = {'success': False, 'msg': ''}
            if robot_name not in self.robots:
                return response
            # Toggle action mode
            if mode.toggle:
                # Use robot mode publisher to set it to "attaching cart mode"
                self.get_logger().info('Publishing attaching mode...')
                msg = self._make_mode_request(
                    robot_name, cmd_id,
                    RobotMode.MODE_PERFORMING_ACTION,
                    'attach_cart')
            else:
                # Use robot mode publisher to set it to "detaching cart mode"
                self.get_logger().info('Publishing detaching mode...')
                msg = self._make_mode_request(
                    robot_name, cmd_id,
                    RobotMode.MODE_PERFORMING_ACTION,
                    'detach_cart')
            self.mode_pub.publish(msg)
            response['success'] = True
            return response

    def _make_mode_request(self, robot_name, cmd_id, mode, action=''):
        mode_msg = ModeRequest()
        mode_msg.fleet_name = self.fleet_name
        mode_msg.robot_name = robot_name
        mode_msg.mode.mode = mode
        mode_msg.mode.mode_request_id = cmd_id
        mode_msg.mode.performing_action = action
        return mode_msg

    # How far past the last reached waypoint to look. This is the bound the
    # adapter used to carry as ACK_LOOKAHEAD, and it is the same trade: too
    # small and a run of vertices the robot slipped past between state messages
    # strands the path, because arrival is judged purely by proximity and a
    # waypoint never reached is only credited via a later one; too large and a
    # route folding back on itself aliases onto a distant waypoint and credits
    # ground never covered. The level guard below rules out the worst aliasing
    # case. It was 3 in the adapter, which stranded paths in simulation.
    LOOKAHEAD = 10

    def advance_reached_index(self, robot: State):
        """
        Move `robot.reached_index` to the furthest waypoint reached.

        Furthest-within-radius, not first-not-reached. Stopping at the first
        waypoint the robot is *not* inside deadlocks the path whenever it slips
        past a vertex between state messages, since nothing else would ever
        credit that one.

        Two bounds, both load-bearing:

        * Stop at the first waypoint on a different level. A lift occupies one
          (x, y) on every floor it serves, so position alone puts a robot in
          the car simultaneously inside that lift's waypoint on every level.
          The hotel has three levels and two lifts, so this is reproducible.
          The path is ordered, so once it leaves this floor nothing beyond it
          can have been reached yet.
        * Look ahead LOOKAHEAD waypoints only.

        Only ever increases: the scan starts past `reached_index` and assigns
        nothing smaller. That matters because the adapter turns this into the
        path index it reports to RMF, which converts a decrease into an
        unclamped itinerary delay.
        """
        with robot.progress_lock:
            waypoints = robot.path_waypoints
            start = robot.reached_index + 1
        if not waypoints or robot.state is None:
            return

        location = robot.state.location
        position = [location.x, location.y]
        limit = min(start + self.LOOKAHEAD, len(waypoints))

        furthest = None
        for i in range(start, limit):
            waypoint = waypoints[i]
            if waypoint['map_name'] != location.level_name:
                break
            if self.disp(position, [waypoint['x'], waypoint['y']]) \
                    <= waypoint['arrival_radius']:
                furthest = i

        if furthest is not None:
            with robot.progress_lock:
                # Re-check under the lock: follow_path may have installed a new
                # path while we were measuring the old one. `is` on the list we
                # sampled is the cheapest way to tell -- a new path is always a
                # new list -- and it keeps the index monotonic even then.
                if robot.path_waypoints is waypoints:
                    robot.reached_index = max(robot.reached_index, furthest)

    def robot_state_cb(self, msg):
        if msg.name in self.robots:
            robot = self.robots[msg.name]
            if (
                not robot.is_expected_task_id(msg.task_id)
                and not robot.perform_action_mode
            ):
                # This message is out of date, so disregard it.
                if robot.last_path_request is not None:
                    # Resend the latest task request for this robot, in case
                    # the message was dropped.
                    if self.debug:
                        print(
                            f'Republishing task request for {msg.name}: '
                            f'{robot.last_path_request.task_id}, '
                            f'because it is currently following {msg.task_id}'
                        )
                    self.path_pub.publish(robot.last_path_request)
                return

            robot.state = msg
            self.advance_reached_index(robot)
            completed_request = None

            if (
                msg.mode.mode == RobotMode.MODE_IDLE
                or msg.mode.mode == RobotMode.MODE_CHARGING
            ) and len(msg.path) == 0 and msg.task_id and msg.task_id.isdigit():
                robot = self.robots[msg.name]
                robot.destination = None
                completed_request = int(msg.task_id)

            # Update completed requests in internal robot state
            if completed_request is not None:
                if robot.last_completed_request != completed_request:
                    if self.debug:
                        print(
                            f'Detecting completed request for {msg.name}: '
                            f'{completed_request}'
                        )
                robot.last_completed_request = completed_request

    def dock_summary_cb(self, msg):
        for fleet in msg.docks:
            if fleet.fleet_name == self.fleet_name:
                for dock in fleet.params:
                    self.docks[dock.start] = dock.path

    def get_robot_state(self, robot: State, robot_name):
        data = {}
        if self.gps:
            position = copy.deepcopy(robot.gps_pos)
        else:
            position = [robot.state.location.x, robot.state.location.y]
        angle = robot.state.location.yaw
        data['robot_name'] = robot_name
        data['map_name'] = robot.state.location.level_name
        data['position'] = {'x': position[0], 'y': position[1], 'yaw': angle}
        data['battery'] = robot.state.battery_percent
        if (
            robot.destination is not None
            and robot.last_path_request is not None
        ):
            destination = robot.destination
            # remove offset for calculation if using gps coords
            if self.gps:
                position[0] -= self.offset[0]
                position[1] -= self.offset[1]
            # calculate arrival estimate
            dist_to_target = self.disp(
                position, [destination.x, destination.y]
            )
            ori_delta = abs(abs(angle) - abs(destination.yaw))
            if ori_delta > np.pi:
                ori_delta = ori_delta - (2 * np.pi)
            if ori_delta < -np.pi:
                ori_delta = (2 * np.pi) + ori_delta
            duration = (
                dist_to_target / self.vehicle_traits.linear.nominal_velocity
                + ori_delta / self.vehicle_traits.rotational.nominal_velocity
            )
            cmd_id = int(robot.last_path_request.task_id)
            data['destination_arrival'] = {
                'cmd_id': cmd_id,
                'duration': duration,
            }
        else:
            data['destination_arrival'] = None

        data['last_completed_request'] = robot.last_completed_request
        # How far along its current path this manager has seen the robot get.
        # The adapter acknowledges against this instead of measuring distances
        # itself -- this side is what actually knows where the robot is, and it
        # sees every state message where the adapter samples at its own rate.
        # \sa advance_reached_index
        with robot.progress_lock:
            data['path_id'] = robot.path_id
            data['reached_waypoint_index'] = robot.reached_index
        if (
            robot.state.mode.mode == RobotMode.MODE_WAITING
            or robot.state.mode.mode == RobotMode.MODE_ADAPTER_ERROR
        ):
            # The name of MODE_WAITING is not very intuitive, but the slotcar
            # plugin uses it to indicate when another robot is blocking its
            # path.
            #
            # MODE_ADAPTER_ERROR means the robot received a plan that
            # didn't make sense, i.e. the plan expected the robot was starting
            # very far from its real present location. When that happens we
            # should replan, so we'll set replan to true in that case as well.
            data['replan'] = True
        else:
            data['replan'] = False
        if (robot.state.mode.mode == RobotMode.MODE_ACTION_COMPLETED):
            self.get_logger().info(
                f'Robot [{robot_name} completed performing its action')
            completed_cmd_id = 0
            msg = self._make_mode_request(robot_name, completed_cmd_id,
                                          RobotMode.MODE_IDLE)
            # Mark action execution as finished
            self.action_completed_pub.publish(msg)
            # # Request for robot idle
            self.mode_pub.publish(msg)

        return data

    def disp(self, A, B):
        return math.sqrt((A[0] - B[0]) ** 2 + (A[1] - B[1]) ** 2)


# -----------------------------------------------------------------------------
# Main
# -----------------------------------------------------------------------------
def main(argv=sys.argv):
    # Init rclpy and adapter
    rclpy.init(args=argv)
    adpt.init_rclcpp()
    args_without_ros = rclpy.utilities.remove_ros_args(argv)

    parser = argparse.ArgumentParser(
        prog='fleet_manager',
        description='Configure and spin up the path guide fleet manager',
    )
    parser.add_argument(
        '-c',
        '--config_file',
        type=str,
        required=True,
        help='Path to the config.yaml file',
    )
    args = parser.parse_args(args_without_ros[1:])
    print('Starting path guide fleet manager...')

    with open(args.config_file, 'r') as f:
        config = yaml.safe_load(f)

    fleet_manager = FleetManager(config)

    spin_thread = threading.Thread(target=rclpy.spin, args=(fleet_manager,))
    spin_thread.start()

    uvicorn.run(
        app,
        host=config['fleet_manager']['ip'],
        port=config['fleet_manager']['port'],
        log_level='warning',
    )


if __name__ == '__main__':
    main(sys.argv)
