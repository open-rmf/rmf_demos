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
import argparse
import yaml
import time
import threading
import asyncio
import math

import rclpy
import rclpy.node
from rclpy.parameter import Parameter
from rclpy.duration import Duration

import rmf_adapter
from rmf_adapter import Adapter
import rmf_adapter.easy_full_control as rmf_easy

from rmf_fleet_msgs.msg import LaneRequest, ClosedLanes, ModeRequest, RobotMode

from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy as History
from rclpy.qos import QoSDurabilityPolicy as Durability
from rclpy.qos import QoSReliabilityPolicy as Reliability
from rclpy.qos import qos_profile_system_default

from .RobotClientAPI import RobotAPI, RobotUpdateData, RobotAPIResult


# ------------------------------------------------------------------------------
# Main
# ------------------------------------------------------------------------------
def main(argv=sys.argv):
    # Init rclpy and adapter
    rclpy.init(args=argv)
    rmf_adapter.init_rclcpp()
    args_without_ros = rclpy.utilities.remove_ros_args(argv)

    parser = argparse.ArgumentParser(
        prog="fleet_adapter",
        description="Configure and spin up the fleet adapter")
    parser.add_argument("-c", "--config_file", type=str, required=True,
                        help="Path to the config.yaml file")
    parser.add_argument("-n", "--nav_graph", type=str, required=True,
                        help="Path to the nav_graph for this fleet adapter")
    parser.add_argument("-sim", "--use_sim_time", action="store_true",
                        help='Use sim time, default: false')
    args = parser.parse_args(args_without_ros[1:])
    print(f"Starting fleet adapter...")

    config_path = args.config_file
    nav_graph_path = args.nav_graph

    fleet_config = rmf_easy.FleetConfiguration.from_config_files(
        config_path, nav_graph_path
    )
    assert fleet_config, f'Failed to parse config file [{config_path}]'

    # Parse the yaml in Python to get the fleet_manager info
    with open(config_path, "r") as f:
        config_yaml = yaml.safe_load(f)

    # ROS 2 node for the command handle
    fleet_name = fleet_config.fleet_name
    node = rclpy.node.Node(f'{fleet_name}_command_handle')
    adapter = Adapter.make(f'{fleet_name}_fleet_adapter')
    assert adapter, (
        'Unable to initialize fleet adapter. '
        'Please ensure RMF Schedule Node is running'
    )

    # Enable sim time for testing offline
    if args.use_sim_time:
        param = Parameter("use_sim_time", Parameter.Type.BOOL, True)
        node.set_parameters([param])
        adapter.node.use_sim_time()

    adapter.start()
    time.sleep(1.0)

    node.declare_parameter('server_uri', '')
    server_uri = node.get_parameter(
        'server_uri'
    ).get_parameter_value().string_value
    if server_uri == '':
        server_uri = None

    fleet_config.server_uri = server_uri
    fleet_handle = adapter.add_easy_fleet(fleet_config)

    # Initialize robot API for this fleet
    fleet_mgr_yaml = config_yaml['fleet_manager']
    update_period = 1.0/fleet_mgr_yaml.get(
        'robot_state_update_frequency', 10.0
    )
    api_prefix = (
        'http://' + fleet_mgr_yaml['ip'] + ':'
        + str(fleet_mgr_yaml['port'])
    )
    api = RobotAPI(
        api_prefix,
        fleet_mgr_yaml['user'],
        fleet_mgr_yaml['password']
    )

    robots = {}
    for robot_name in fleet_config.known_robots:
        robot_config = fleet_config.get_known_robot_configuration(robot_name)
        robots[robot_name] = RobotAdapter(
            robot_name, robot_config, node, api, fleet_handle
        )

    def update_loop():
        asyncio.set_event_loop(asyncio.new_event_loop())
        while rclpy.ok():
            now = node.get_clock().now()

            # Update all the robots in parallel using a thread pool
            update_jobs = []
            for robot in robots.values():
                update_jobs.append(update_robot(robot))

            asyncio.get_event_loop().run_until_complete(
                asyncio.wait(update_jobs)
            )

            next_wakeup = now + Duration(nanoseconds=update_period*1e9)
            while node.get_clock().now() < next_wakeup:
                time.sleep(0.001)

    update_thread = threading.Thread(target=update_loop, args=())
    update_thread.start()

    # Connect to the extra ROS2 topics that are relevant for the adapter
    ros_connections(node, robots, fleet_handle)

    # Create executor for the command handle node
    rclpy_executor = rclpy.executors.SingleThreadedExecutor()
    rclpy_executor.add_node(node)

    # Start the fleet adapter
    rclpy_executor.spin()

    # Shutdown
    node.destroy_node()
    rclpy_executor.shutdown()
    rclpy.shutdown()


class RobotAdapter:
    def __init__(
        self,
        name: str,
        configuration,
        node,
        api: RobotAPI,
        fleet_handle
    ):
        self.name = name
        self.execution = None
        self.teleoperation = None
        self.cmd_id = 0
        self.update_handle = None
        self.configuration = configuration
        self.node = node
        self.api = api
        self.fleet_handle = fleet_handle
        self.override = None
        self.issue_cmd_thread = None
        self.cancel_cmd_event = threading.Event()

    def update(self, state, data: RobotUpdateData):
        activity_identifier = None
        if self.execution:
            if data.is_command_completed(self.cmd_id):
                self.execution.finished()
                self.execution = None
                self.teleoperation = None
            else:
                activity_identifier = self.execution.identifier

        if self.teleoperation is not None:
            self.teleoperation.update(data)

        self.update_handle.update(state, activity_identifier)

    def make_callbacks(self):
        return rmf_easy.RobotCallbacks(
            lambda destination, execution: self.navigate(
                destination, execution
            ),
            lambda activity: self.stop(activity),
            lambda category, description, execution: self.execute_action(
                category, description, execution
            )
        )

    def navigate(self, destination, execution):
        self.cmd_id += 1
        self.execution = execution
        self.node.get_logger().info(
            f'Commanding [{self.name}] to navigate to {destination.position} '
            f'on map [{destination.map}]: cmd_id {self.cmd_id}'
        )

        if destination.dock is not None:
            self.attempt_cmd_until_success(
                cmd=self.perform_docking,
                args=(destination,)
            )
            return

        self.attempt_cmd_until_success(
            cmd=self.api.navigate,
            args=(
                self.name,
                self.cmd_id,
                destination.position,
                destination.map,
                destination.speed_limit
            )
        )

    def stop(self, activity):
        if self.execution is not None:
            if self.execution.identifier.is_same(activity):
                self.execution = None
                self.attempt_cmd_until_success(
                    cmd=self.api.stop,
                    args=(self.name, self.cmd_id)
                )

    def execute_action(self, category: str, description: dict, execution):
        self.cmd_id += 1
        self.execution = execution

        match category:
            case 'teleop':
                self.teleoperation = Teleoperation(execution)
                self.attempt_cmd_until_success(
                    cmd=self.api.toggle_teleop,
                    args=(self.name, True)
                )
            case 'clean':
                self.attempt_cmd_until_success(
                    cmd=self.perform_clean,
                    args=(description['zone'],)
                )

    def finish_action(self):
        # This is triggered by a ModeRequest callback which allows human
        # operators to manually change the operational mode of the robot. This
        # is typically used to indicate when teleoperation has finished.
        if self.execution is not None:
            self.execution.finished()
            self.execution = None
            self.attempt_cmd_until_success(
                cmd=self.api.toggle_teleop,
                args=(self.name, False)
            )

    def perform_docking(self, destination):
        match self.api.start_activity(
            self.name,
            self.cmd_id,
            'dock',
            destination.dock()
        ):
            case (RobotAPIResult.SUCCESS, path):
                self.override = self.execution.override_schedule(
                    path['map_name'],
                    path['path']
                )
                return True
            case RobotAPIResult.RETRY:
                return False
            case RobotAPIResult.IMPOSSIBLE:
                # If the fleet manager does not know this dock name, then treat
                # it as a regular navigation request
                return self.api.navigate(
                    self.name,
                    self.cmd_id,
                    destination.position,
                    destination.map,
                    destination.speed_limit
                )

    def perform_clean(self, zone):
        match self.api.start_activity(self.name, self.cmd_id, 'clean', zone):
            case (RobotAPIResult.SUCCESS, path):
                self.node.get_logger().info(
                    f'Commanding [{self.name}] to clean zone [{zone}]'
                )
                self.override = self.execution.override_schedule(
                    path['map_name'],
                    path['path']
                )
                return True
            case RobotAPIResult.RETRY:
                return False
            case RobotAPIResult.IMPOSSIBLE:
                self.node.get_logger().error(
                    f'Fleet manager for [{self.name}] does not know how to '
                    f'clean zone [{zone}]. We will terminate the activity.'
                )
                self.execution.finished()
                self.execution = None
                return True

    def attempt_cmd_until_success(self, cmd, args):
        self.cancel_cmd_attempt()

        def loop():
            while not cmd(*args):
                self.node.get_logger().warn(
                    f'Failed to contact fleet manager for robot {self.name}'
                )
                if self.cancel_cmd_event.wait(1.0):
                    break

        self.issue_cmd_thread = threading.Thread(
            target=loop,
            args=()
        )
        self.issue_cmd_thread.start()

    def cancel_cmd_attempt(self):
        if self.issue_cmd_thread is not None:
            self.cancel_cmd_event.set()
            if self.issue_cmd_thread.is_alive():
                self.issue_cmd_thread.join()
                self.issue_cmd_thread = None
        self.cancel_cmd_event.clear()


class Teleoperation:
    def __init__(self, execution):
        self.execution = execution
        self.override = None
        self.last_position = None

    def update(self, data: RobotUpdateData):
        if self.last_position is None:
            print(
                'about to override schedule with '
                f'{data.map}: {[data.position]}'
            )
            self.override = self.execution.override_schedule(
                data.map, [data.position], 30.0
            )
            self.last_position = data.position
        else:
            dx = self.last_position[0] - data.position[0]
            dy = self.last_position[1] - data.position[1]
            dist = math.sqrt(dx*dx + dy*dy)
            if dist > 0.1:
                print('about to replace override schedule')
                self.override = self.execution.override_schedule(
                    data.map, [data.position], 30.0
                )
                self.last_position = data.position


# Parallel processing solution derived from
# https://stackoverflow.com/a/59385935
def parallel(f):
    def run_in_parallel(*args, **kwargs):
        return asyncio.get_event_loop().run_in_executor(
            None, f, *args, **kwargs
        )

    return run_in_parallel


@parallel
def update_robot(robot: RobotAdapter):
    data = robot.api.get_data(robot.name)
    if data is None:
        return

    state = rmf_easy.RobotState(
        data.map,
        data.position,
        data.battery_soc
    )

    if robot.update_handle is None:
        robot.update_handle = robot.fleet_handle.add_robot(
            robot.name,
            state,
            robot.configuration,
            robot.make_callbacks()
        )
        return

    robot.update(state, data)


def ros_connections(node, robots, fleet_handle):
    fleet_name = fleet_handle.more().fleet_name

    transient_qos = QoSProfile(
        history=History.KEEP_LAST,
        depth=1,
        reliability=Reliability.RELIABLE,
        durability=Durability.TRANSIENT_LOCAL
    )

    closed_lanes_pub = node.create_publisher(
        ClosedLanes,
        'closed_lanes',
        qos_profile=transient_qos
    )

    closed_lanes = set()

    def lane_request_cb(msg):
        if msg.fleet_name is None or msg.fleet_name != fleet_name:
            return

        fleet_handle.open_lanes(msg.open_lanes)
        fleet_handle.close_lanes(msg.close_lanes)

        for lane_idx in msg.close_lanes:
            closed_lanes.add(lane_idx)

        for lane_idx in msg.open_lanes:
            closed_lanes.remove(lane_idx)

        state_msg = ClosedLanes()
        state_msg.fleet_name = fleet_name
        state_msg.closed_lanes = list(closed_lanes)
        closed_lanes_pub.publish(state_msg)

    def mode_request_cb(msg):
        if (
            msg.fleet_name is None
            or msg.fleet_name != fleet_name
            or msg.robot_name is None
        ):
            return

        if msg.mode.mode == RobotMode.MODE_IDLE:
            robot = robots.get(msg.robot_name)
            if robot is None:
                return
            robot.finish_action()

    node.create_subscription(
        LaneRequest,
        'lane_closure_requests',
        lane_request_cb,
        qos_profile=qos_profile_system_default
    )

    node.create_subscription(
        ModeRequest,
        'action_execution_notice',
        mode_request_cb,
        qos_profile=qos_profile_system_default
    )


if __name__ == '__main__':
    main(sys.argv)
