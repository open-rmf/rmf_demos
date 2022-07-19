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


from flask_cors import CORS
from flask import Flask, request, jsonify
import rclpy
import rclpy.executors
import threading
import sys
import json
import copy
import argparse
import logging
from collections import OrderedDict
from rosidl_runtime_py import message_to_ordereddict
from pyproj import Transformer

from rclpy.node import Node
from flask_socketio import SocketIO, emit, disconnect

from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy as History
from rclpy.qos import QoSDurabilityPolicy as Durability
from rclpy.qos import QoSReliabilityPolicy as Reliability

from rmf_fleet_msgs.msg import FleetState, RobotState

log = logging.getLogger('werkzeug')
log.disabled = True

SUPPORTED_GPS_FRAMES = ['svy21']

# Temporary Message definition for GPS messages
GPS_MESSAGE_DEFINITION = {
    "timestamp": -1,     # Unix time
    "robot_id": "",      # String
    "lat": -1,           # Float32
    "lon": -1,           # Float32
    "alt": -1,           # Float32
    "heading": 0,        # Uint

    "x": 0,              # 2D robot x position
    "y": 0,              # 2D robot y position
    "angle": 0,          # 2D robot angle

    "battery": 100.0,    # Battery percentage
}


class FleetSocketIOBridge(Node):
    def __init__(self, argv=sys.argv):
        parser = argparse.ArgumentParser()
        parser.add_argument('-t', '--robot_state_topic',
                            required=False,
                            type=str,
                            default='/robot_state',
                            help='Topic to listen on for Robot States')
        parser.add_argument('-f', '--filter_fleet',
                            required=False,
                            type=str,
                            help='Only listen to this fleet.')
        parser.add_argument('-g', '--gps_state_topic',
                            required=False,
                            type=str,
                            help='SocketIO topic to publish GPS.')
        parser.add_argument('-x', '--offset_x',
                            required=False,
                            type=float,
                            default=0.0,
                            help='X offset for simulations.')
        parser.add_argument('-y', '--offset_y',
                            required=False,
                            type=float,
                            default=0.0,
                            help='Y offset for simulations.')
        parser.add_argument('-i', '--listening_interfaces',
                            required=False,
                            type=str,
                            default='0.0.0.0',
                            help='Interfaces for SocketIO to serve on')
        parser.add_argument('-p', '--listening_port',
                            required=False,
                            type=str,
                            default='8080',
                            help='Port for SocketIO to serve on')

        self.args = parser.parse_args(argv[1:])

        super().__init__(f"fleet_socketio_bridge")

        self._init_pubsub()
        self._init_webserver()

        if self.args.gps_state_topic:
            self._init_gps_conversion_tools('svy21')

    def robot_state_callback(self, msg: RobotState):
        try:
            if self.args.filter_fleet:
                if self.args.filter_fleet not in msg.name:
                    return

            self._sio.emit(self.args.robot_state_topic,
                           message_to_ordereddict(msg))

            if self.args.gps_state_topic:
                robot_state_json = self._robot_state_to_gps_json(msg)
                self._sio.emit(self.args.gps_state_topic,
                               robot_state_json)
        except Exception as e:
            print(e)

    def start_socketio(self):
        self._app.run(self.args.listening_interfaces, self.args.listening_port)

    def spin_background(self):
        def spin():
            self.get_logger().info("start spinning rclpy node")
            rclpy.spin_until_future_complete(self, self._finish_spin)
            self.get_logger().info("Finished spinning")
        self._spin_thread = threading.Thread(target=spin)
        self._spin_thread.start()

    def _init_webserver(self):
        self._finish_spin = rclpy.executors.Future()
        self._finish_gc = self.create_guard_condition(
            lambda: self._finish_spin.set_result(None))

        self._app = Flask(__name__)
        CORS(self._app, origins=r"/*")

        self._sio = SocketIO(self._app, async_mode='threading')
        self._sio.init_app(self._app, cors_allowed_origins="*")

    def _init_pubsub(self):
        self.robot_state_sub = self.create_subscription(
            RobotState,
            self.args.robot_state_topic,
            self.robot_state_callback,
            10)

    def _init_gps_conversion_tools(self, frame: str):
        assert frame in SUPPORTED_GPS_FRAMES

        if frame == 'svy21':
            self._wgs_transformer = Transformer.from_crs(
                'EPSG:3414', 'EPSG:4326')
            return

        raise Exception("This should not happen")

    def _robot_state_to_gps_json(self, robot_state: RobotState):
        resp = copy.deepcopy(GPS_MESSAGE_DEFINITION)

        svy21_xy = self._remove_offsets(
            robot_state.location.x,
            robot_state.location.y
        )
        wgs84_xy = self._wgs_transformer.transform(
            svy21_xy[1], svy21_xy[0])  # inputs are y,x
        resp["timestamp"] = robot_state.location.t.sec
        resp["robot_id"] = robot_state.name
        resp["lat"] = wgs84_xy[0]
        resp["lon"] = wgs84_xy[1]
        resp["alt"] = 0         # BH(WARN): Hardcoded
        resp["heading"] = robot_state.location.yaw

        resp["x"] = robot_state.location.x
        resp["y"] = robot_state.location.y
        resp["angle"] = robot_state.location.yaw

        resp["battery"] = robot_state.battery_percent

        return json.dumps(resp)

    def _remove_offsets(self, x: float, y: float):
        return (x + self.args.offset_x,
                y + self.args.offset_y)

    def _apply_offsets(self, x: float, y: float):
        return (x - self.args.offset_x,
                y - self.args.offset_y)


def main(argv=sys.argv):
    rclpy.init(args=argv)

    node = FleetSocketIOBridge()
    try:
        node.get_logger().info('calling spin()')
        node.spin_background()
        node.start_socketio()
    except KeyboardInterrupt:
        pass
    except OSError:
        node.get_logger().error(
            "Check if target port is already in use.")
    finally:
        node.get_logger().info('shutting down...')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main(sys.argv)
