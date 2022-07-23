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
import paho.mqtt.client as mqtt

SUPPORTED_GPS_FRAMES = ['svy21']

ROBOT_ID_TO_AUTHKEY_MAP = {
    "deliveryRobot_1": "00000000-0000-0000-0000-000000000001",
    "deliveryRobot_2": "00000000-0000-0000-0000-000000000002",
    "deliveryRobot_3": "00000000-0000-0000-0000-000000000003",
}


ROBOTMANAGER_GPS_DEFINITION = {
    "batteryPct": 100.0,
    "mapPose": {
        "x": 1076.0,
        "y": 456.0,
        "z": 0.0,
        "heading": 0.0
    },
    "globalPose": {
        "lat": 1.30032,
        "lng": 103.78063,
        "alt": 0.0,
        "heading": 0.0
    },
    "state": 3
}


class FleetRobotManagerMQTTBridge(Node):
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
        parser.add_argument('-g', '--mqtt_base_topic',
                            required=False,
                            type=str,
                            default="/robot/status/",
                            help='MQTT base topic to publish GPS.')
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
        parser.add_argument('-m', '--mqtt_server',
                            required=False,
                            type=str,
                            default="localhost",
                            help='MQTT server url')

        self.args = parser.parse_args(argv[1:])

        super().__init__(f"fleet_robotmanager_mqtt_bridge")

        self._init_pubsub()
        self.mqtt_pubs = {}    # Map of robot names to their mqtt Client
        self._init_mqtt()
        self._init_gps_conversion_tools('svy21')

    def robot_state_callback(self, msg: RobotState):
        try:
            if self.args.filter_fleet:
                if self.args.filter_fleet not in msg.name:
                    return
            robot = msg
            if robot.name not in ROBOT_ID_TO_AUTHKEY_MAP.keys():
                print(
                    f"Skipping {robot.name} as it does not have auth key")
            else:
                json = self._robot_state_to_gps_json(robot)
                rbmgr_uuid = ROBOT_ID_TO_AUTHKEY_MAP[robot.name]
                self.mqtt_pubs[robot.name].publish(
                    self.args.mqtt_base_topic + rbmgr_uuid,
                    json
                )

        except Exception as e:
            print(e)

    def _init_pubsub(self):
        self.robot_state_sub = self.create_subscription(
            RobotState,
            self.args.robot_state_topic,
            self.robot_state_callback,
            10)

    def _init_mqtt(self):
        try:
            for robot_name in ROBOT_ID_TO_AUTHKEY_MAP.keys():
                self.mqtt_pubs[robot_name] = mqtt.Client(
                    'rbmgr_pub_' + ROBOT_ID_TO_AUTHKEY_MAP[robot_name])
                self.mqtt_pubs[robot_name].connect(self.args.mqtt_server)
        except ConnectionRefusedError as e:
            print(f"MQTT connection to {self.args.mqtt_server} failed.")
            print("Please check that the MQTT server is running!")
            raise(e)

    def _init_gps_conversion_tools(self, frame: str):
        assert frame in SUPPORTED_GPS_FRAMES

        if frame == 'svy21':
            self._wgs_transformer = Transformer.from_crs(
                'EPSG:3414', 'EPSG:4326')
            return

        raise Exception("This should not happen")

    def _robot_state_to_gps_json(self, robot_state: RobotState):
        resp = copy.deepcopy(ROBOTMANAGER_GPS_DEFINITION)

        svy21_xy = self._remove_offsets(
            robot_state.location.x,
            robot_state.location.y
        )
        wgs84_xy = self._wgs_transformer.transform(
            svy21_xy[1], svy21_xy[0])  # inputs are y,x
        resp["timestamp"] = robot_state.location.t.sec
        resp["robot_id"] = robot_state.name
        resp["globalPose"]["lat"] = wgs84_xy[0]
        resp["globalPose"]["lng"] = wgs84_xy[1]
        resp["globalPose"]["alt"] = 0
        resp["globalPose"]["heading"] = robot_state.location.yaw

        resp["mapPose"]["x"] = robot_state.location.x
        resp["mapPose"]["y"] = robot_state.location.y
        resp["mapPose"]["z"] = 0.0
        resp["mapPose"]["heading"] = robot_state.location.yaw

        resp["batteryPct"] = robot_state.battery_percent

        return json.dumps(resp)

    def _remove_offsets(self, x: float, y: float):
        return (x + self.args.offset_x,
                y + self.args.offset_y)

    def _apply_offsets(self, x: float, y: float):
        return (x - self.args.offset_x,
                y - self.args.offset_y)


def main(argv=sys.argv):
    rclpy.init(args=argv)

    node = FleetRobotManagerMQTTBridge()
    try:
        node.get_logger().info('calling spin()')
        rclpy.spin(node)
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
