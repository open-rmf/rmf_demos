import sys
import argparse
import yaml
import json
import math
import threading
from icecream import ic
import time
from copy import deepcopy
import paho.mqtt.client as mqtt

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from urllib.error import HTTPError

import requests


class RobotUpdateData:
    def __init__(self, data):
        self.robot_name = data['robot_name']
        position = data['position']
        x = position['x']
        y = position['y']
        yaw = position['yaw']
        self.position = [x, y, yaw]
        self.map = data['map_name']
        self.battery_soc = data['battery'] / 100.0
        self.requires_replan = data.get('replan', False)
        self.last_request_completed = data['last_completed_request']

    def is_command_completed(self, cmd_id):
        return self.last_request_completed == cmd_id


class ReadOnlyPublisher(Node):
    def __init__(self, config, states_json):
        self.name = 'read_only_publisher'
        super().__init__(self.name)

        self.timeout = 10.0
        self.connected = False
        self.last_publish_time = None
        self.update_interval = 4.0

        fleet_mgr_config = config['fleet_manager']
        self.prefix = (
            'http://' + fleet_mgr_config['ip'] + ':' + str(fleet_mgr_config['port'])
        )

        # TODO(@xiyuoh) support multiple robots
        self.robot_name = states_json['robots'][0]['name']
        self.cmd_id = 0
        self.waypoints = states_json['robots'][0]['waypoints']
        self.waypoints_copy = deepcopy(self.waypoints)

        # Set up MQTT to receive robot states
        self.start_mqtt_client(config['mqtt'])

    def start_mqtt_client(self, mqtt_config):
        self.mqtt_topic = mqtt_config['topic']['fleet_manager']

        def _on_message(client, user_data, msg):
            pass

        def _on_connect(client, userdata, flags, rc):
            if rc == 0:
                print(f'[{self.name}] connected to MQTT broker!')
                self.connected = True
            else:
                print(f'[{self.name}] failed to connect, return code {rc}')

        self.mqtt_client = mqtt.Client(self.name)
        if (mqtt_config['connect'].get('username') is not None and
                mqtt_config['connect'].get('password') is not None):
            self.mqtt_client.username_pw_set(
                mqtt_config['connect']['username'],
                mqtt_config['connect']['password'])
        self.mqtt_client.on_connect = _on_connect
        self.mqtt_client.on_message = _on_message
        self.mqtt_client.connect(mqtt_config['connect']['server_url'])
        self.mqtt_client.loop_start()

        # Test initial connectivity
        self.get_logger().info('Checking connectivity.')
        count = 0
        while not self.check_connection():
            if count >= self.timeout:
                print('Unable to connect to robot API.')
                break
            else:
                print('Unable to connect to robot API. Attempting to reconnect...')
                count += 1
            time.sleep(1)
        if not self.check_connection():
            self.get_logger().error('Failed to establish connection with robot API')
            sys.exit(1)

        # We are connected! Let's start publishing robot states
        self.create_timer(0.2, self.read_only_handler)

    def check_connection(self) -> bool:
        if not self.connected:
            return False
        return True

    def read_only_handler(self):
        '''
        Handles the navigation commands to fleet manager and publishes robot
        state to read_only_driver
        '''
        if len(self.waypoints) < 2:
            self.waypoints = deepcopy(self.waypoints)
            return

        data = self.get_data()
        if not data:
            return

        if (not data.last_request_completed and self.cmd_id == 0) or \
                data.is_command_completed(self.cmd_id):
            # Clear previous point
            self.waypoints.pop(0)
            # Send command to the next state
            self.cmd_id += 1
            next_point = self.waypoints[0]
            self.navigate(
                self.cmd_id,
                [next_point['x'], next_point['y'], next_point['yaw']],
                data.map
            )

        current_state = {
            'robots': [{
                'name': 1,
                'battery_soc': 1,
                'position': {
                    'x': data.position[0],
                    'y': data.position[1],
                    'yaw': data.position[2],
                    'map': data.map
                },
                'waypoints': self.waypoints,
            }]
        }

        msg = json.dumps(current_state)
        self.mqtt_client.publish(self.mqtt_topic, msg)

    def get_data(self):
        url = (
            self.prefix
            + f'/open-rmf/rmf_demos_fm/status?robot_name={self.robot_name}'
        )
        try:
            response = requests.get(url, timeout=self.timeout)
            response.raise_for_status()
            return RobotUpdateData(response.json()['data'])
        except HTTPError as http_err:
            print(f'HTTP error for {self.robot_name} in get_data: {http_err}')
        except Exception as err:
            print(f'Other error for {self.robot_name} in get_data: {err}')
        return None

    def navigate(
        self,
        cmd_id: int,
        pose,
        map_name: str,
        speed_limit=0.0,
    ):
        assert len(pose) > 2
        url = (
            self.prefix
            + f'/open-rmf/rmf_demos_fm/navigate?robot_name={self.robot_name}'
            f'&cmd_id={cmd_id}'
        )
        data = {}  # data fields: task, map_name, destination{}, data{}
        data['map_name'] = map_name
        data['destination'] = {'x': pose[0], 'y': pose[1], 'yaw': pose[2]}
        data['speed_limit'] = speed_limit
        try:
            response = requests.post(url, timeout=self.timeout, json=data)
            response.raise_for_status()
            return response.json()['success']
        except HTTPError as http_err:
            print(f'HTTP error for {self.robot_name} in navigate: {http_err}')
        except Exception as err:
            print(f'Other error for {self.robot_name} in navigate: {err}')
        return False

    def disp(self, A, B):
        return math.sqrt((A[0] - B[0]) ** 2 + (A[1] - B[1]) ** 2)

def main(argv=sys.argv):
    # Init rclpy
    rclpy.init(args=argv)
    args_without_ros = rclpy.utilities.remove_ros_args(argv)

    parser = argparse.ArgumentParser(
        prog="read_only_publisher",
        description="Configure and spin up the read only state publisher")
    parser.add_argument("-c", "--config_file", type=str, required=True,
                        help="Path to the config.yaml file")
    parser.add_argument("-r", "--robot_states_file", type=str, required=True,
                        help="Path to the robot states json")
    parser.add_argument("-sim", "--use_sim_time", action="store_true",
                        help='Use sim time, default: false')
    args = parser.parse_args(args_without_ros[1:])
    print(f"Starting read only publisher...")

    with open(args.config_file, "r") as f:
        config = yaml.safe_load(f)
    with open(args.robot_states_file, "r") as f:
        states_json = json.load(f)

    publisher = ReadOnlyPublisher(config, states_json)
    # Enable sim time for testing offline
    if args.use_sim_time:
        param = Parameter("use_sim_time", Parameter.Type.BOOL, True)
        publisher.set_parameters([param])

    spin_thread = threading.Thread(target=rclpy.spin, args=(publisher,))
    spin_thread.start()


if __name__ == '__main__':
    main(sys.argv)