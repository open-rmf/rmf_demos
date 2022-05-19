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
import nudged
import time

import rclpy
import rclpy.node
from rclpy.parameter import Parameter

import rmf_adapter as adpt
import rmf_adapter.vehicletraits as traits
import rmf_adapter.battery as battery
import rmf_adapter.geometry as geometry
import rmf_adapter.graph as graph
import rmf_adapter.plan as plan

from rmf_task_msgs.msg import TaskProfile, TaskType

from functools import partial

from .RobotCommandHandle import RobotCommandHandle

# ------------------------------------------------------------------------------
# Helper functions
# ------------------------------------------------------------------------------


def initialize_fleet(config_yaml, nav_graph_path, node, use_sim_time):
    # Profile and traits
    fleet_config = config_yaml['rmf_fleet']
    profile = traits.Profile(geometry.make_final_convex_circle(
        fleet_config['profile']['footprint']),
        geometry.make_final_convex_circle(fleet_config['profile']['vicinity']))
    vehicle_traits = traits.VehicleTraits(
        linear=traits.Limits(*fleet_config['limits']['linear']),
        angular=traits.Limits(*fleet_config['limits']['angular']),
        profile=profile)
    vehicle_traits.differential.reversible = fleet_config['reversible']

    # Battery system
    voltage = fleet_config['battery_system']['voltage']
    capacity = fleet_config['battery_system']['capacity']
    charging_current = fleet_config['battery_system']['charging_current']
    battery_sys = battery.BatterySystem.make(
        voltage, capacity, charging_current)

    # Mechanical system
    mass = fleet_config['mechanical_system']['mass']
    moment = fleet_config['mechanical_system']['moment_of_inertia']
    friction = fleet_config['mechanical_system']['friction_coefficient']
    mech_sys = battery.MechanicalSystem.make(mass, moment, friction)

    # Power systems
    ambient_power_sys = battery.PowerSystem.make(
        fleet_config['ambient_system']['power'])
    tool_power_sys = battery.PowerSystem.make(
        fleet_config['tool_system']['power'])

    # Power sinks
    motion_sink = battery.SimpleMotionPowerSink(battery_sys, mech_sys)
    ambient_sink = battery.SimpleDevicePowerSink(
        battery_sys, ambient_power_sys)
    tool_sink = battery.SimpleDevicePowerSink(battery_sys, tool_power_sys)

    nav_graph = graph.parse_graph(nav_graph_path, vehicle_traits)

    # Adapter
    fleet_name = fleet_config['name']
    adapter = adpt.Adapter.make(f'{fleet_name}_fleet_adapter')
    if use_sim_time:
        adapter.node.use_sim_time()
    assert adapter, ("Unable to initialize fleet adapter. Please ensure "
                     "RMF Schedule Node is running")
    adapter.start()
    time.sleep(1.0)

    fleet_handle = adapter.add_fleet(fleet_name, vehicle_traits, nav_graph)

    if not fleet_config['publish_fleet_state']:
        fleet_handle.fleet_state_publish_period(None)
    # Account for battery drain
    drain_battery = fleet_config['account_for_battery_drain']
    recharge_threshold = fleet_config['recharge_threshold']
    recharge_soc = fleet_config['recharge_soc']
    finishing_request = fleet_config['task_capabilities']['finishing_request']
    node.get_logger().info(f"Finishing request: [{finishing_request}]")
    # Set task planner params
    ok = fleet_handle.set_task_planner_params(
        battery_sys,
        motion_sink,
        ambient_sink,
        tool_sink,
        recharge_threshold,
        recharge_soc,
        drain_battery,
        finishing_request)
    assert ok, ("Unable to set task planner params")

    task_capabilities = []
    if fleet_config['task_capabilities']['loop']:
        node.get_logger().info(
            f"Fleet [{fleet_name}] is configured to perform Loop tasks")
        task_capabilities.append(TaskType.TYPE_LOOP)
    if fleet_config['task_capabilities']['delivery']:
        node.get_logger().info(
            f"Fleet [{fleet_name}] is configured to perform Delivery tasks")
        task_capabilities.append(TaskType.TYPE_DELIVERY)
    if fleet_config['task_capabilities']['clean']:
        node.get_logger().info(
            f"Fleet [{fleet_name}] is configured to perform Clean tasks")
        task_capabilities.append(TaskType.TYPE_CLEAN)

    # Callable for validating requests that this fleet can accommodate
    def _task_request_check(task_capabilities, msg: TaskProfile):
        if msg.description.task_type in task_capabilities:
            return True
        else:
            return False

    fleet_handle.accept_task_requests(
        partial(_task_request_check, task_capabilities))

    # Transforms
    rmf_coordinates = config_yaml['reference_coordinates']['rmf']
    robot_coordinates = config_yaml['reference_coordinates']['robot']
    transforms = {
        'rmf_to_robot': nudged.estimate(rmf_coordinates, robot_coordinates),
        'robot_to_rmf': nudged.estimate(robot_coordinates, rmf_coordinates)}
    transforms['orientation_offset'] = \
        transforms['rmf_to_robot'].get_rotation()
    mse = nudged.estimate_error(transforms['rmf_to_robot'],
                                rmf_coordinates,
                                robot_coordinates)
    print(f"Coordinate transformation error: {mse}")
    print("RMF to Robot transform:")
    print(f"    rotation:{transforms['rmf_to_robot'].get_rotation()}")
    print(f"    scale:{transforms['rmf_to_robot'].get_scale()}")
    print(f"    trans:{transforms['rmf_to_robot'].get_translation()}")
    print("Robot to RMF transform:")
    print(f"    rotation:{transforms['robot_to_rmf'].get_rotation()}")
    print(f"    scale:{transforms['robot_to_rmf'].get_scale()}")
    print(f"    trans:{transforms['robot_to_rmf'].get_translation()}")

    def _updater_inserter(cmd_handle, update_handle):
        """Insert a RobotUpdateHandle."""
        cmd_handle.update_handle = update_handle

    # Initialize robots for this fleet
    robots = {}
    for robot_name, robot_config in config_yaml['robots'].items():
        node.get_logger().info(f"Initializing robot:{robot_name}")
        rmf_config = robot_config['rmf_config']
        robot_config = robot_config['robot_config']
        initial_waypoint = rmf_config['start']['waypoint']
        initial_orientation = rmf_config['start']['orientation']
        robot = RobotCommandHandle(
            name=robot_name,
            config=robot_config,
            node=node,
            graph=nav_graph,
            vehicle_traits=vehicle_traits,
            transforms=transforms,
            map_name=rmf_config['start']['map_name'],
            initial_waypoint=initial_waypoint,
            initial_orientation=initial_orientation,
            charger_waypoint=rmf_config['charger']['waypoint'],
            update_frequency=rmf_config.get('robot_state_update_frequency', 1),
            adapter=adapter)

        if robot.initialized:
            robots[robot_name] = robot
            # Add robot to fleet
            fleet_handle.add_robot(robot,
                                   robot_name,
                                   profile,
                                   robot.starts,
                                   partial(_updater_inserter, robot))
            node.get_logger().info(
                f"Successfully added new robot:{robot_name}")

        else:
            node.get_logger().error(
                f"Failed to initialize robot:{robot_name}")

    return adapter, fleet_handle, robots


# ------------------------------------------------------------------------------
# Main
# ------------------------------------------------------------------------------
def main(argv=sys.argv):
    # Init rclpy and adapter
    rclpy.init(args=argv)
    adpt.init_rclcpp()
    print(f"Starting fleet adapter...")

    # ROS 2 node for the command handle
    node = rclpy.node.Node('robot_command_handle')

    node.declare_parameter("config_file", rclpy.Parameter.Type.STRING)
    node.declare_parameter("nav_graph_file", rclpy.Parameter.Type.STRING)

    arg_use_sim_time = node.get_parameter(
        'use_sim_time').get_parameter_value().bool_value
    assert arg_use_sim_time is not None
    arg_config_path = node.get_parameter(
        'config_file').get_parameter_value().string_value
    assert arg_config_path is not None
    arg_nav_graph_path = node.get_parameter(
        'nav_graph_file').get_parameter_value().string_value
    assert arg_nav_graph_path is not None

    # Load config and nav graph yamls
    with open(arg_config_path, "r") as f:
        config_yaml = yaml.safe_load(f)

    adapter, fleet_handle, robots = initialize_fleet(
        config_yaml,
        arg_nav_graph_path,
        node,
        arg_use_sim_time)

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
