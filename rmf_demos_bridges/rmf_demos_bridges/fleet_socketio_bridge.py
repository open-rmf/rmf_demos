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
import argparse
from collections import OrderedDict
from rosidl_runtime_py import message_to_ordereddict

from rclpy.node import Node
from flask_socketio import SocketIO, emit, disconnect

from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy as History
from rclpy.qos import QoSDurabilityPolicy as Durability
from rclpy.qos import QoSReliabilityPolicy as Reliability

from rmf_fleet_msgs.msg import FleetState


class FleetSocketIOBridge(Node):
    def __init__(self, argv=sys.argv):
        parser = argparse.ArgumentParser()
        parser.add_argument('-t', '--fleet_state_topic',
                            required=False,
                            type=str,
                            default='/fleet_states',
                            help='Topic to listen on for Fleet States')
        parser.add_argument('-i', '--listening_interfaces',
                            required=False,
                            type=str,
                            default='0.0.0.0',
                            help='Interfaces for SocketIO to listen on')
        parser.add_argument('-p', '--listening_port',
                            required=False,
                            type=str,
                            default='8080',
                            help='Port for SocketIO to serve on')

        self.args = parser.parse_args(argv[1:])

        super().__init__(f"fleet_socketio_bridge")

        self._init_pubsub()
        self._init_webserver()

    def fleet_state_callback(self, msg: FleetState):
        try:
            self.socketio.emit(msg.name, message_to_ordereddict(msg))
        except Exception as e:
            print(e)

    def start_socketio(self):
        self.app.run(self.args.listening_interfaces, self.args.listening_port)

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

        self.app = Flask(__name__)
        CORS(self.app, origins=r"/*")

        self.socketio = SocketIO(self.app, async_mode='threading')
        self.socketio.init_app(self.app, cors_allowed_origins="*")

    def _init_pubsub(self):
        self.fleet_state_sub = self.create_subscription(
            FleetState,
            self.args.fleet_state_topic,
            self.fleet_state_callback,
            10)


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
