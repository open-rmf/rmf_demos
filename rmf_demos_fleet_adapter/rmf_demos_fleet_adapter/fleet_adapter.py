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
from multiprocessing import Pool

import rclpy
import rclpy.node
from rclpy.parameter import Parameter
from rclpy.duration import Duration

import rmf_adapter
import rmf_adapter.Adapter as Adapter
import rmf_adapter.easy_full_control as rmf_easy

from rmf_fleet_msgs.msg import LaneRequest, ClosedLanes, ModeRequest, RobotMode

from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy as History
from rclpy.qos import QoSDurabilityPolicy as Durability
from rclpy.qos import QoSReliabilityPolicy as Reliability
from rclpy.qos import qos_profile_system_default

from .RobotCommandHandle import RobotCommandHandle
from .RobotClientAPI import RobotAPI, RobotUpdateData

from rmf_fleet_msgs.msg import DockSummary

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
    fleet_name = fleet_config.name
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

    node.declare_paramter('server_uri', rclpy.Parameter.Type.STRING)
    server_uri = node.get_parameter(
        'server_uri'
    ).get_parameter_value().string_value
    if server_uri == '':
        server_uri = None

    fleet_config.server_uri = server_uri
    fleet_handle = adapter.add_easy_fleet(fleet_config)

    # Initialize robot API for this fleet
    fleet_mgr_yaml = config_yaml['fleet_manager']
    update_period = 1.0/fleet_mgr_yaml['robot_state_update_frequency']
    api_prefix = (
        'http://' + fleet_mgr_yaml['ip'] + ':'
        + str(fleet_mgr_yaml['port'])
    )
    api = RobotAPI(
        api_prefix,
        fleet_mgr_yaml['user'],
        fleet_mgr_yaml['password']
    )

    docks = {}
    robots = []
    for robot_name in fleet_config.known_robots:
        robot_config = fleet_config.get_known_robot_configuration(robot_name)
        robots.append(RobotAdapter(robot_name, robot_config, node, api, docks))

    def update_robot(robot: RobotAdapter):
        data = api.get_data(robot.name)
        if data is None:
            return

        state = rmf_easy.RobotState(
            data.map,
            data.position,
            data.battery_soc
        )

        if robot.update_handle is None:
            robot.update_handle = fleet_handle.add_robot(
                robot.name,
                state,
                robot.configuration,
                robot.make_callbacks
            )
            return

        robot.update(state)


    def update_loop():
        pool = Pool(4)
        while rclpy.ok():
            now = node.get_clock().now()

            # Update all the robots in parallel using a thread pool
            pool.map(update_robot, robots)

            next_wakeup = now + Duration(nanoseconds=update_period*1e9)
            while node.get_clock().now() < next_wakeup:
                time.sleep(0.001)

    update_thread = threading.Thread(target=update_loop, args=())
    update_thread.start()

    # Connect to the extra ROS2 topics that are relevant for the adapter
    ros_connections(node, robots, docks, fleet_handle)

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
        docks
    ):
        self.name = name
        self.execution = None
        self.activity_identifier = None
        self.cmd_id = 0
        self.update_handle = None
        self.configuration = configuration
        self.node = node
        self.api = api
        self.docks = docks
        self.override = None
        self.issue_cmd_thread = None
        self.cancel_cmd_event = threading.Event()

    def update(self, state: RobotUpdateData):
        if self.execution:
            if state.is_command_completed(self.cmd_id):
                self.execution.finished()
                self.execution = None
                self.activity_identifier = None

        self.update_handle.update(state, self.activity_identifier)


    def make_callbacks(self):
        rmf_easy.RobotCallbacks(
            lambda destination, execution: self.navigate(destination, execution),
            lambda: self.stop(),
            lambda category, description, execution: self.execute_action(category, description, execution)
        )

    def navigate(self, destination, execution):
        self.execution = execution
        self.activity_identifier = execution.identifier
        self.cmd_id += 1

        if destination.dock() is not None:
            path = self.docks.get(destination.dock())
            if path is not None:
                self.perform_docking(destination, execution, path)
                return
            # If we don't recognize the dock name then just treat this as a
            # regular navigation command.

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

    def stop(self):
        self.current_activity = None
        self.attempt_cmd_until_success(
            cmd=self.api.stop,
            args=(self.name, self.cmd_id)
        )

    def execute_action(self, category: str, description: dict, execution):
        self.execution = execution
        self.activity_identifier = execution.identifier


    def finish_action(self):
        # This is triggered by a ModeRequest callback which allows human
        # operators to manually change the operational mode of the robot. This
        # is typically used to indicate when teleoperation has finished.
        if self.execution is not None:
            self.execution.finished()
            self.execution = None
            self.activity_identifier = None


    def perform_docking(self, destination, execution, path):
        self.override = execution.override_schedule(
            destination.map(),
            path
        )

        self.attempt_cmd_until_success(
            cmd=self.api.start_process,
            args=(
                self.name,
                self.cmd_id,
                destination.dock(),
                destination.map()
            )
        )

    def attempt_cmd_until_success(self, cmd, args):
        self.cancel_cmd_attempt()
        def loop():
            while not cmd(args):
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


def ros_connections(node, robots, docks, fleet_handle):
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

    def dock_summary_cb(msg):
        # TODO(@mxgrey): The DockSummary messages are inconsistent with how
        # docks are labelled. They use a `start` and `finish` waypoint
        # representation instead of a lane representation.
        for fleet in msg.docks:
            if fleet.fleet_name != fleet_name:
                continue
            for dock in fleet.params:
                path = []
                for p in dock.path:
                    path.append([p.x, p.y, p.yaw])
                docks[dock.start] = path

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
        if msg.fleet_name is None or msg.fleet_name != fleet_name or msg.robot_name is None:
            return

        if msg.mode.mode == RobotMode.MODE_IDLE:
            robot = robots.get(msg.robot_name)
            if robot is None:
                return
            robot.finish_action()


    node.create_subscription(
        DockSummary,
        'dock_summary',
        dock_summary_cb,
        qos_profile=transient_qos
    )

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
