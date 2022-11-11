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
import datetime
import enum

import rclpy
import rclpy.node
from rclpy.parameter import Parameter

import rmf_adapter as adpt
import rmf_adapter.plan as plan
import rmf_adapter.schedule as schedule

from rmf_fleet_msgs.msg import DockSummary, ModeRequest, LaneRequest, \
    ClosedLanes

from functools import partial

from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy as History
from rclpy.qos import QoSDurabilityPolicy as Durability
from rclpy.qos import QoSReliabilityPolicy as Reliability
from rclpy.qos import qos_profile_system_default

from .RobotClientAPI import RobotAPI

# ------------------------------------------------------------------------------
# Helper functions
# ------------------------------------------------------------------------------


# States for RobotCommandHandle's state machine used when guiding robot along
# a new path
class RobotState(enum.IntEnum):
    IDLE = 0
    MOVING = 1


class FleetAdapter:

    def __init__(self, config_path, nav_graph_path, node, use_sim_time):
        # global counter for command ids
        self.next_id = 0
        # Keep track of which map the robot is in
        self.last_map = {}
        self.cmd_ids = {}
        # Load config yaml
        with open(config_path, "r") as f:
            config_yaml = yaml.safe_load(f)
        # Initialize robot API for this fleet
        fleet_config = config_yaml['rmf_fleet']
        prefix = 'http://' + fleet_config['fleet_manager']['ip'] + \
                 ':' + str(fleet_config['fleet_manager']['port'])
        self.api = RobotAPI(
            prefix,
            fleet_config['fleet_manager']['user'],
            fleet_config['fleet_manager']['password'])

        node.declare_parameter('server_uri', rclpy.Parameter.Type.STRING)
        server_uri = node.get_parameter(
            'server_uri').get_parameter_value().string_value
        if server_uri == "":
            server_uri = None

        # ----------------------------------
        # RMF Demos specific items
        # ----------------------------------
        self.fleet_name = fleet_config['name']
        self.node = node
        self.closed_lanes = []  # Store the closed lanes in the map
        self.docks = {}  # Map robots to docks
        self.actions = {}  # Map robots to actions

        transient_qos = QoSProfile(
            history=History.KEEP_LAST,
            depth=1,
            reliability=Reliability.RELIABLE,
            durability=Durability.TRANSIENT_LOCAL)
        # Dock summary subscriber
        node.create_subscription(
            DockSummary,
            'dock_summary',
            self.dock_summary_cb,
            qos_profile=transient_qos)
        # Subscribe to ModeRequest for action execution notice
        node.create_subscription(
            ModeRequest,
            'action_execution_notice',
            self.mode_request_cb,
            qos_profile=qos_profile_system_default)
        # Lane request subscriber
        node.create_subscription(
            LaneRequest,
            'lane_closure_requests',
            self.lane_request_cb,
            qos_profile=qos_profile_system_default)
        # Closed lanes publisher
        self.closed_lanes_pub = node.create_publisher(
            ClosedLanes,
            'closed_lanes',
            qos_profile=transient_qos)

        # Create EasyFullControl adapter
        self.configuration = adpt.easy_full_control.Configuration.make(
            config_path, nav_graph_path, server_uri)
        self.adapter = self.initialize_fleet(
            self.configuration, config_yaml['robots'], node, use_sim_time)

    def initialize_fleet(self, configuration, robots_yaml, node, use_sim_time):
        # Make the easy full control
        easy_full_control = adpt.EasyFullControl.make(configuration)

        if use_sim_time:
            easy_full_control.node.use_sim_time()

        def _goal_completed(robot_name):
            success = self.api.process_completed(
                robot_name, self.cmd_ids[robot_name])
            request_replan = self.api.requires_replan(robot_name)
            remaining_time = self.api.navigation_remaining_duration(
                robot_name, self.cmd_ids[robot_name])
            if remaining_time:
                remaining_time = datetime.timedelta(seconds=remaining_time)
            goal_status = adpt.easy_full_control.GoalStatus(
                success,
                remaining_time,
                request_replan)
            return goal_status

        def _robot_state(robot_name):
            data = self.api.data(robot_name)
            if data is None or data['success'] is False:
                return None
            pos = data['data']['position']
            action = False
            if robot_name in self.actions and \
                    self.actions[robot_name] is not None:
                action = True
            state = adpt.easy_full_control.RobotState(
                robot,
                robot_config['charger']['waypoint'],
                data['data']['map_name'],
                [pos['x'], pos['y'], pos['yaw']],
                data['data']['battery'],
                action)
            self.last_map[robot_name] = data['data']['map_name']
            return state

        def _navigate(robot_name, map_name, goal, update_handle):
            cmd_id = self.next_id
            self.next_id += 1
            self.cmd_ids[robot_name] = cmd_id
            self.api.navigate(robot_name, cmd_id, goal, map_name)
            node.get_logger().info(f"Navigating robot {robot_name}")
            return partial(_goal_completed, robot_name)

        def _stop(robot_name):
            cmd_id = self.next_id
            self.next_id += 1
            self.cmd_ids[robot_name] = cmd_id
            return self.api.stop(robot_name, cmd_id)

        def _dock(robot_name, dock_name, update_handle):
            if dock_name not in self.docks:
                node.get_logger().info(
                    f'Requested dock {dock_name} not found, '
                    f'ignoring docking request')
                return

            cmd_id = self.next_id
            self.next_id += 1
            self.cmd_ids[robot_name] = cmd_id
            self.api.start_process(
                robot_name, cmd_id, dock_name, self.last_map[robot_name])

            positions = []
            for wp in self.docks[dock_name]:
                positions.append([wp.x, wp.y, wp.yaw])
            task_completed_cb = self.api.navigation_completed
            node.get_logger().info(
                f"Robot {robot_name} is docking at {dock_name}...")
            self.traj_thread = threading.Thread(
                target=self.start_trajectory,
                args=(task_completed_cb, robot_name, positions,
                      update_handle))
            self.traj_thread.start()

            return partial(_goal_completed, robot_name)

        def _action_executor(
                robot_name: str,
                category: str,
                description: dict,
                execution: adpt.robot_update_handle.ActionExecution):
            self.actions[robot_name] = execution
            self.api.toggle_action(robot_name, True)

        # Add the robots
        for robot in robots_yaml:
            node.get_logger().info(f'Found robot {robot}')
            robot_config = robots_yaml[robot]['rmf_config']
            success = False
            while success is False:
                state = _robot_state(robot)
                if state is None:
                    time.sleep(0.2)
                    continue
                success = True
                # Add robot to fleet
                easy_full_control.add_robot(
                    state,
                    partial(_robot_state, robot),
                    partial(_navigate, robot),
                    partial(_stop, robot),
                    partial(_dock, robot),
                    partial(_action_executor, robot))

        return easy_full_control

    # Store docking paths
    def dock_summary_cb(self, msg):
        for fleet in msg.docks:
            if (fleet.fleet_name == self.fleet_name):
                for dock in fleet.params:
                    self.docks[dock.start] = dock.path

    # Receive action execution notice to end robot action
    def mode_request_cb(self, msg):
        if msg.fleet_name is None or msg.fleet_name != self.fleet_name or\
                msg.robot_name is None:
            return
        if msg.mode.mode == RobotState.IDLE:
            self.complete_robot_action(msg.robot_name)

    # Complete robot action
    def complete_robot_action(self, robot_name):
        action_execution = self.actions[robot_name]
        if action_execution is None:
            return
        action_execution.finished()
        self.api.toggle_action(robot_name, False)
        self.actions[robot_name] = None
        self.node.get_logger().info(f'Robot {robot_name} has completed the'
                                    f' action it was performing')

    # Track newly closed lanes
    def lane_request_cb(self, msg):
        if msg.fleet_name is None or msg.fleet_name != self.fleet_name:
            return
        self.adapter.fleet_handle().open_lanes(msg.open_lanes)
        self.adapter.fleet_handle().close_lanes(msg.close_lanes)
        newly_closed_lanes = []
        for lane_idx in msg.close_lanes:
            if lane_idx not in self.closed_lanes:
                newly_closed_lanes.append(lane_idx)
                self.closed_lanes.append(lane_idx)

        for lane_idx in msg.open_lanes:
            if lane_idx in self.closed_lanes:
                self.closed_lanes.remove(lane_idx)

        self.adapter.newly_closed_lanes(set(newly_closed_lanes))

        state_msg = ClosedLanes()
        state_msg.fleet_name = self.fleet_name
        state_msg.closed_lanes = self.closed_lanes
        self.closed_lanes_pub.publish(state_msg)

    # Track trajectory of docking or custom actions
    def start_trajectory(self,
                         task_completed_cb,
                         robot_name,
                         positions,
                         update_handle):
        while not task_completed_cb(robot_name, self.cmd_ids[robot_name]):
            now = datetime.datetime.fromtimestamp(0) + \
                self.adapter.node.now()
            traj = schedule.make_trajectory(
                self.configuration.vehicle_traits(),
                now,
                positions)
            itinerary = schedule.Route(self.last_map[robot_name], traj)
            if update_handle is not None:
                participant = update_handle.get_unstable_participant()
                participant.set_itinerary([itinerary])


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
    parser.add_argument("-sim", "--use_sim_time", action="store_true",
                        help='Use sim time, default: false')
    args = parser.parse_args(args_without_ros[1:])
    print(f"Starting fleet adapter...")

    config_path = args.config_file
    nav_graph_path = args.nav_graph

    # Load config and nav graph yamls
    with open(config_path, "r") as f:
        config_yaml = yaml.safe_load(f)

    # ROS 2 node for the command handle
    fleet_name = config_yaml['rmf_fleet']['name']
    node = rclpy.node.Node(f'{fleet_name}_command_handle')

    # Enable sim time for testing offline
    if args.use_sim_time:
        param = Parameter("use_sim_time", Parameter.Type.BOOL, True)
        node.set_parameters([param])

    adapter = FleetAdapter(
        config_path,
        nav_graph_path,
        node,
        args.use_sim_time)

    # Create executor for the command handle node
    rclpy_executor = rclpy.executors.SingleThreadedExecutor()
    rclpy_executor.add_node(node)

    # Start the fleet adapter
    rclpy_executor.spin()

    # Shutdown
    node.destroy_node()
    rclpy_executor.shutdown()
    rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv)
