# Copyright 2020 Open Source Robotics Foundation, Inc.
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
The main API Interfaces (with port 8080):
1) HTTP interfaces are:  /submit_task, /cancel_task, /task_list, /robot_list
2) socketIO broadcast states: /task_status, /robot_states, /ros_time
"""

import sys
import os
import rclpy
import argparse
import time
import json
import logging
from threading import Thread

from flask import Flask, request, jsonify
from flask_cors import CORS
from flask_socketio import SocketIO, emit, disconnect

from rmf_demos_panel.dispatcher_client import DispatcherClient

###############################################################################


app = Flask(__name__)
cors = CORS(app, origins=r"/*")

socketio = SocketIO(app, async_mode='threading')
socketio.init_app(app, cors_allowed_origins="*")

rclpy.init(args=None)
dispatcher_client = DispatcherClient()

# logging config
logging.getLogger('werkzeug').setLevel(logging.ERROR)  # hide logs from flask
logging.basicConfig(level=logging.DEBUG,
                    format='%(asctime)s %(levelname)s %(message)s',
                    filename='web_server.log',
                    filemode='w')

###############################################################################


@app.route('/submit_task', methods=['POST'])
def submit():
    """REST Call to submit task"""
    task_id, err_msg = dispatcher_client.submit_task_request(request.json)
    logging.debug(f" ROS Time: {dispatcher_client.ros_time()} | \
        Task Submission: {json.dumps(request.json)}, error: {err_msg}")
    return jsonify({"task_id": task_id, "error_msg": err_msg})


@app.route('/cancel_task', methods=['POST'])
def cancel():
    cancel_id = request.json['task_id']
    cancel_success = dispatcher_client.cancel_task_request(cancel_id)
    logging.debug(f" ROS Time: {dispatcher_client.ros_time()} | \
        Cancel Task: {cancel_id}, success: {cancel_success}")
    return jsonify({"success": cancel_success})


@app.route('/task_list', methods=['GET'])
def status():
    task_status = jsonify(dispatcher_client.get_task_status())
    logging.debug(f" ROS Time: {dispatcher_client.ros_time()} | \
        Task Status: {json.dumps(task_status.json)}")
    return task_status


@app.route('/robot_list', methods=['GET'])
def robots():
    robot_status = jsonify(dispatcher_client.get_robot_states())
    logging.debug(f" ROS Time: {dispatcher_client.ros_time()} | \
        Robot Status: {robot_status}")
    return robot_status


@app.route('/building_map', methods=['GET'])
def building_map():
    building_map_data = jsonify(dispatcher_client.get_building_map_data())
    logging.debug(f" ROS Time: {dispatcher_client.ros_time()} | \
        building_map_data: {building_map_data}")
    return building_map_data

###############################################################################


def web_server_spin():
    while rclpy.ok():
        dispatcher_client.spin_once()
        time.sleep(0.2)


def broadcast_states():
    """
    Robot_states, tasks_status, and ros_time are being broadcasted
    to frontend UIs via socketIO, periodically (every 2s)
    """
    ns = '/status_updates'
    while rclpy.ok():
        with app.test_request_context():
            tasks = dispatcher_client.get_task_status()
            robots = dispatcher_client.get_robot_states()
            ros_time = dispatcher_client.ros_time()
            socketio.emit('task_status', tasks, broadcast=True, namespace=ns)
            socketio.emit('robot_states', robots, broadcast=True, namespace=ns)
            socketio.emit('ros_time', ros_time, broadcast=True, namespace=ns)
            logging.debug(f" ROS Time: {ros_time} | "
                          "active tasks: "
                          f"{len(dispatcher_client.active_tasks_cache)}"
                          " | terminated tasks: "
                          f"{len(dispatcher_client.terminated_tasks_cache)}"
                          f" | active robots: {len(robots)}")
        time.sleep(2)

###############################################################################


def main(args=None):
    server_ip = "0.0.0.0"
    port_num = 8080

    if "WEB_SERVER_IP_ADDRESS" in os.environ:
        server_ip = os.environ['WEB_SERVER_IP_ADDRESS']
        print(f"Set Server IP to: {server_ip}:{port_num}")

    spin_thread = Thread(target=web_server_spin, args=())
    spin_thread.start()

    broadcast_thread = Thread(target=broadcast_states, args=())
    broadcast_thread.start()

    print("Starting Dispatcher API Server")
    app.run(host=server_ip, port=port_num, debug=False)
    dispatcher_client.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main(sys.argv)
