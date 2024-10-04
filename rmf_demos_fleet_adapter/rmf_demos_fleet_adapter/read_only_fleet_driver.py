import sys
import argparse
import yaml
import threading
import json
import time
import nudged
import numpy as np
import paho.mqtt.client as mqtt

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

import rmf_adapter
from rmf_adapter import Transformation

from rmf_fleet_msgs.msg import FleetState, RobotState, Location

from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy as History
from rclpy.qos import QoSDurabilityPolicy as Durability
from rclpy.qos import QoSReliabilityPolicy as Reliability


class FleetDriver(Node):
    def __init__(self, config):
        self.fleet_name = config['rmf_fleet']['name']
        super().__init__(f'{self.fleet_name}_fleet_driver')

        # Set up map conversions
        self.rmf_to_robot_map = {}
        self.robot_to_rmf_map = {}
        for rmf_map, robot_map in config['conversions']['map'].items():
            self.robot_to_rmf_map[robot_map] = rmf_map
            self.rmf_to_robot_map[rmf_map] = robot_map

        # Set up transforms for each level
        self.transforms = {}
        for level, coords in config['conversions']['reference_coordinates'].items():
            # These are robot to RMF transforms
            self.transforms[level] = self.compute_transforms(level, coords)

        volatile_qos = QoSProfile(
            history=History.KEEP_LAST,
            depth=100,
            reliability=Reliability.RELIABLE,
            durability=Durability.VOLATILE)

        self.fleet_state_pub = self.create_publisher(
            FleetState,
            'fleet_states',
            qos_profile=volatile_qos)

        # Set up MQTT to receive robot states
        self.timeout = 10.0
        self.connected = False
        self.start_mqtt_client(config['mqtt'])

    def start_mqtt_client(self, mqtt_config):
        fleet_mqtt_topic = mqtt_config['topic']['fleet_manager']
        mqtt_sub_topics = []
        mqtt_sub_topics.append((fleet_mqtt_topic, 0))

        def _on_message(client, user_data, msg):
            if msg.topic == fleet_mqtt_topic:
                payload = json.loads(msg.payload.decode())
                if 'robots' in payload:
                    self.update_robot_state(payload)

        def _on_connect(client, userdata, flags, rc):
            if rc == 0:
                print(f'[{self.fleet_name}] connected to MQTT broker!')
                self.connected = True
            else:
                print(f'[{self.fleet_name}] failed to connect, return code {rc}')

        self.mqtt_client = mqtt.Client(self.fleet_name)
        if (mqtt_config['connect'].get('username') is not None and
                mqtt_config['connect'].get('password') is not None):
            self.mqtt_client.username_pw_set(
                mqtt_config['connect']['username'],
                mqtt_config['connect']['password'])
        self.mqtt_client.on_connect = _on_connect
        self.mqtt_client.on_message = _on_message
        self.mqtt_client.connect(mqtt_config['connect']['server_url'])
        self.mqtt_client.subscribe(mqtt_sub_topics)
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

    def check_connection(self) -> bool:
        if not self.connected:
            return False
        return True

    def update_robot_state(self, robot_states):
        payload_robots = robot_states['robots']

        fleet_state_msg = FleetState()
        robots = []

        for robot in payload_robots:
            # Robot current pose
            current_pose = robot['position']
            current_robot_map = current_pose['map']
            current_rmf_map = self.robot_to_rmf_map[current_robot_map]
            # Transform from robot coords to RMF coords
            robot_pose = np.array([current_pose['x'], current_pose['y'], current_pose['yaw']])
            map_transform = self.transforms[current_rmf_map]
            rmf_pose = map_transform.apply(robot_pose)
            # Create RobotState
            robot_state = RobotState()
            robot_state.name = robot['name']
            robot_state.battery_percent = robot['battery_soc']
            location = Location()
            location.t = self.get_clock().now().to_msg()
            location.x = rmf_pose[0]
            location.y = rmf_pose[1]
            location.yaw = rmf_pose[2]
            location.level_name = current_rmf_map
            robot_state.location = location

            # Robot path
            robot_state.path = []
            waypoints = robot['waypoints']
            for pt in waypoints:
                pt_robot_map = pt['map']
                pt_rmf_map = self.robot_to_rmf_map[pt_robot_map]
                pt_pose = np.array([pt['x'], pt['y'], pt['yaw']])
                rmf_pt_pose = map_transform.apply(pt_pose)
                pt_loc = Location()
                pt_loc.x = rmf_pt_pose[0]
                pt_loc.y = rmf_pt_pose[1]
                pt_loc.yaw = rmf_pt_pose[2]
                pt_loc.level_name = pt_rmf_map

                robot_state.path.append(pt_loc)

            robots.append(robot_state)

        # Publish fleet state
        fleet_state_msg.name = self.fleet_name
        fleet_state_msg.robots = robots
        self.fleet_state_pub.publish(fleet_state_msg)

    def compute_transforms(self, level, coords):
        rotation = None
        scale = None
        translation = None
        if 'rmf' in coords:
            rmf_coords = coords['rmf']
            robot_coords = coords[self.fleet_name]
            # Robot to RMF transform
            tf = nudged.estimate(robot_coords, rmf_coords)
            mse = nudged.estimate_error(tf, robot_coords, rmf_coords)
            self.get_logger().info(
                f"Transformation error estimate for {level}: {mse}"
            )
            rotation = tf.get_rotation()
            scale = tf.get_scale()
            translation = tf.get_translation()
        elif 'rotation' in coords:
            rotation = coords['rotation']
            scale = coords['scale']
            translation = coords['translation']
        else:
            return None

        return Transformation(
            rotation,
            scale,
            translation
        )


def main(argv=sys.argv):
    # Init rclpy
    rclpy.init(args=argv)
    args_without_ros = rclpy.utilities.remove_ros_args(argv)

    parser = argparse.ArgumentParser(
        prog="rmf_read_only_driver",
        description="Configure and spin up the fleet driver")
    parser.add_argument("-c", "--config_file", type=str, required=True,
                        help="Path to the config.yaml file")
    parser.add_argument("-sim", "--use_sim_time", action="store_true",
                        help='Use sim time, default: false')
    args = parser.parse_args(args_without_ros[1:])
    print(f"Starting fleet driver...")

    with open(args.config_file, "r") as f:
        config = yaml.safe_load(f)

    fleet_driver = FleetDriver(config)
    # Enable sim time for testing offline
    if args.use_sim_time:
        param = Parameter("use_sim_time", Parameter.Type.BOOL, True)
        fleet_driver.set_parameters([param])

    spin_thread = threading.Thread(target=rclpy.spin, args=(fleet_driver,))
    spin_thread.start()


if __name__ == '__main__':
    main(sys.argv)