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

import rclpy
import rclpy.node
from rclpy.duration import Duration

from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy as History
from rclpy.qos import QoSDurabilityPolicy as Durability
from rclpy.qos import QoSReliabilityPolicy as Reliability
from rclpy.qos import qos_profile_system_default

import rmf_adapter as adpt
import rmf_adapter.plan as plan
import rmf_adapter.schedule as schedule

from rmf_fleet_msgs.msg import DockSummary, ModeRequest

import numpy as np

import threading
import math
import copy
import enum
import time

from datetime import timedelta


# States for RobotCommandHandle's state machine used when guiding robot along
# a new path
class RobotState(enum.IntEnum):
    IDLE = 0
    WAITING = 1
    MOVING = 2


# Custom wrapper for Plan::Waypoint. We use this to modify position of
# waypoints to prevent backtracking
class PlanWaypoint:
    def __init__(self, index, wp: plan.Waypoint):
        # the index of the Plan::Waypoint in the waypoints in follow_new_path
        self.index = index
        self.position = wp.position
        self.time = wp.time
        self.graph_index = wp.graph_index
        self.approach_lanes = wp.approach_lanes


class RobotCommandHandle(adpt.RobotCommandHandle):
    def __init__(self,
                 name,
                 fleet_name,
                 config,
                 node,
                 graph,
                 vehicle_traits,
                 map_name,
                 start,
                 position,
                 charger_waypoint,
                 update_frequency,
                 adapter,
                 api):
        adpt.RobotCommandHandle.__init__(self)
        self.name = name
        self.fleet_name = fleet_name
        self.config = config
        self.node = node
        self.graph = graph
        self.vehicle_traits = vehicle_traits
        self.map_name = map_name
        # Get the index of the charger waypoint
        waypoint = self.graph.find_waypoint(charger_waypoint)
        assert waypoint, f"Charger waypoint {charger_waypoint} \
          does not exist in the navigation graph"
        self.charger_waypoint_index = waypoint.index
        self.update_frequency = update_frequency
        self.update_handle = None  # RobotUpdateHandle
        self.battery_soc = 1.0
        self.api = api
        self.position = position  # (x,y,theta) in RMF crs (meters,radians)
        self.initialized = False
        self.state = RobotState.IDLE
        self.dock_name = ""
        # TODO(YV): Remove self.adapter. This is only being used for time point
        # comparison with Plan::Waypoint::time
        self.adapter = adapter
        self.action_execution = None

        self.requested_waypoints = []  # RMF Plan waypoints
        self.remaining_waypoints = []
        self.path_finished_callback = None
        self.next_arrival_estimator = None
        self.path_index = 0
        self.docking_finished_callback = None
        self.docks = {}

        # RMF location trackers
        self.last_known_lane_index = None
        self.last_known_waypoint_index = None
        # if robot is waiting at a waypoint. This is a Graph::Waypoint index
        self.on_waypoint = None
        # if robot is travelling on a lane. This is a Graph::Lane index
        self.on_lane = None
        self.target_waypoint = None  # this is a Plan::Waypoint
        # The graph index of the waypoint the robot is currently docking into
        self.dock_waypoint_index = None
        # The graph index of the waypoint the robot starts or ends an action
        self.action_waypoint_index = None

        # Threading variables
        self._lock = threading.Lock()
        self._follow_path_thread = None
        self._quit_path_event = threading.Event()
        self._dock_thread = None
        self._quit_dock_event = threading.Event()

        self.node.get_logger().info(
            f"The robot is starting at: [{self.position[0]:.2f}, "
            f"{self.position[1]:.2f}, {self.position[2]:.2f}]")

        # Update tracking variables
        if start.lane is not None:  # If the robot is on a lane
            self.last_known_lane_index = start.lane
            self.on_lane = start.lane
            self.last_known_waypoint_index = start.waypoint
        else:  # Otherwise, the robot is on a waypoint
            self.last_known_waypoint_index = start.waypoint
            self.on_waypoint = start.waypoint

        transient_qos = QoSProfile(
            history=History.KEEP_LAST,
            depth=1,
            reliability=Reliability.RELIABLE,
            durability=Durability.TRANSIENT_LOCAL)

        self.node.create_subscription(
            DockSummary,
            'dock_summary',
            self.dock_summary_cb,
            qos_profile=transient_qos)

        self.node.create_subscription(
            ModeRequest,
            'action_execution_notice',
            self.mode_request_cb,
            qos_profile=qos_profile_system_default)

        self.update_thread = threading.Thread(target=self.update)
        self.update_thread.start()

        self.initialized = True

    def sleep_for(self, seconds):
        goal_time =\
          self.node.get_clock().now() + Duration(nanoseconds=1e9*seconds)
        while (self.node.get_clock().now() <= goal_time):
            time.sleep(0.001)

    def clear(self):
        with self._lock:
            self.requested_waypoints = []
            self.remaining_waypoints = []
            self.state = RobotState.IDLE

    def stop(self):
        # Stop the robot. Tracking variables should remain unchanged.
        while True:
            self.node.get_logger().info(f"Requesting {self.name} to stop...")
            if self.api.stop(self.name):
                break
            self.sleep_for(0.1)

    def follow_new_path(
            self,
            waypoints,
            next_arrival_estimator,
            path_finished_callback):

        if self._follow_path_thread is not None:
            self._quit_path_event.set()
            if self._follow_path_thread.is_alive():
                self._follow_path_thread.join()
            self._follow_path_thread = None
            self.clear()
        self._quit_path_event.clear()

        self.node.get_logger().info(f"Received new path for {self.name}")

        wait, entries = self.filter_waypoints(waypoints)
        self.remaining_waypoints = copy.copy(entries)
        assert next_arrival_estimator is not None
        assert path_finished_callback is not None
        self.next_arrival_estimator = next_arrival_estimator
        self.path_finished_callback = path_finished_callback

        # Make the robot wait at its current position
        if (wait is not None):
            self.path_index = wait.index
            self.target_waypoint = wait
            self.state = RobotState.WAITING
            with self._lock:
                if (self.target_waypoint.graph_index is not None):
                    self.on_waypoint = self.target_waypoint.graph_index
                else:
                    self.on_waypoint = None  # we are still on a lane
                self.last_known_waypoint_index = self.on_waypoint

        def _follow_path():
            target_pose = []
            while (
                    self.remaining_waypoints or
                    self.state == RobotState.MOVING or
                    self.state == RobotState.WAITING):
                # Check if we need to abort
                if self._quit_path_event.is_set():
                    self.node.get_logger().info(f"Robot [{self.name}] aborting"
                                                " previously followed path")
                    return
                # State machine
                if self.state == RobotState.IDLE:
                    # Assign the next waypoint
                    self.target_waypoint = self.remaining_waypoints[0]
                    self.path_index = self.remaining_waypoints[0].index
                    # Move robot to next waypoint
                    target_pose = self.target_waypoint.position
                    [x, y] = target_pose[:2]
                    theta = target_pose[2]
                    speed_limit = self.get_speed_limit(self.target_waypoint)
                    response = self.api.navigate(self.name,
                                                 [x, y, theta],
                                                 self.map_name,
                                                 speed_limit)

                    if response:
                        self.remaining_waypoints = self.remaining_waypoints[1:]
                        self.state = RobotState.MOVING
                    else:
                        self.node.get_logger().info(
                            f"Robot {self.name} failed to navigate to "
                            f"[{x:.0f}, {y:.0f}, {theta:.0f}] coordinates. "
                            f"Retrying...")
                        self.sleep_for(0.1)

                elif self.state == RobotState.WAITING:
                    self.sleep_for(0.1)
                    time_now = self.adapter.now()
                    with self._lock:
                        if self.target_waypoint is not None:
                            waypoint_wait_time = self.target_waypoint.time
                            if (waypoint_wait_time < time_now):
                                self.state = RobotState.IDLE
                            else:
                                if self.path_index is not None:
                                    d = (waypoint_wait_time - time_now).seconds
                                    self.node.get_logger().debug(
                                        f"Waiting for {d}s")
                                    self.next_arrival_estimator(
                                        self.path_index,
                                        timedelta(seconds=0.0))

                elif self.state == RobotState.MOVING:
                    self.sleep_for(0.1)
                    # Check if we have reached the target
                    with self._lock:
                        if (self.api.navigation_completed(self.name)):
                            self.node.get_logger().info(
                                f"Robot [{self.name}] has reached its target "
                                f"waypoint")
                            self.state = RobotState.WAITING
                            if (self.target_waypoint.graph_index is not None):
                                self.on_waypoint = \
                                    self.target_waypoint.graph_index
                                self.last_known_waypoint_index = \
                                    self.on_waypoint
                            else:
                                self.on_waypoint = None  # still on a lane
                        else:
                            # Update the lane the robot is on
                            lane = self.get_current_lane()
                            if lane is not None:
                                self.on_waypoint = None
                                self.on_lane = lane
                            else:
                                # The robot may either be on the previous
                                # waypoint or the target one
                                if self.target_waypoint.graph_index is not \
                                        None and self.dist(self.position,
                                                           target_pose) < 0.5:
                                    self.on_waypoint =\
                                        self.target_waypoint.graph_index
                                elif self.last_known_waypoint_index is not \
                                        None and self.dist(
                                        self.position, self.graph.get_waypoint(
                                        self.last_known_waypoint_index
                                        ).location) < 0.5:
                                    self.on_waypoint =\
                                        self.last_known_waypoint_index
                                else:
                                    self.on_lane = None  # update_off_grid()
                                    self.on_waypoint = None
                        duration =\
                            self.api.navigation_remaining_duration(self.name)
                        if self.path_index is not None:
                            target_time = self.target_waypoint.time
                            now = self.adapter.now()
                            if target_time < now + timedelta(seconds=duration):
                                self.next_arrival_estimator(
                                    self.path_index,
                                    timedelta(seconds=duration))
                            else:
                                self.next_arrival_estimator(
                                    self.path_index, target_time - now)
            self.path_finished_callback()
            self.node.get_logger().info(
                f"Robot {self.name} has successfully navigated along "
                f"requested path.")

        self._follow_path_thread = threading.Thread(
            target=_follow_path)
        self._follow_path_thread.start()

    def dock(
            self,
            dock_name,
            docking_finished_callback):
        ''' Docking is very specific to each application. Hence, the user will
            need to customize this function accordingly. In this example, we
            assume the dock_name is the same as the name of the waypoints that
            the robot is trying to dock into. We then call api.start_process()
            to initiate the robot specific process. This could be to start a
            cleaning process or load/unload a cart for delivery.
        '''

        self._quit_dock_event.clear()
        if self._dock_thread is not None:
            self._dock_thread.join()

        self.dock_name = dock_name
        assert docking_finished_callback is not None
        self.docking_finished_callback = docking_finished_callback

        # Get the waypoint that the robot is trying to dock into
        dock_waypoint = self.graph.find_waypoint(self.dock_name)
        assert(dock_waypoint)
        self.dock_waypoint_index = dock_waypoint.index

        def _dock():
            # Request the robot to start the relevant process
            while (not self.api.start_process(
                    self.name, self.dock_name, self.map_name)):
                self.node.get_logger().info(
                    f"Requesting robot {self.name} "
                    f"to dock at {self.dock_name}")
                self.sleep_for(1.0)

            with self._lock:
                self.on_waypoint = None
                self.on_lane = None
            self.sleep_for(0.1)

            if self.dock_name not in self.docks:
                self.node.get_logger().info(f"Request dock not found, "
                                            "aborting docking")
                return

            positions = []
            for wp in self.docks[self.dock_name]:
                positions.append([wp.x, wp.y, wp.yaw])
            self.node.get_logger().info(f"Robot {self.name} is docking...")

            while (not self.api.process_completed(self.name)):

                if len(positions) < 1:
                    continue

                traj = schedule.make_trajectory(self.vehicle_traits,
                                                self.adapter.now(),
                                                positions)
                itinerary = schedule.Route(self.map_name, traj)
                if self.update_handle is not None:
                    participant = self.update_handle.get_unstable_participant()
                    participant.set_itinerary([itinerary])

                # Check if we need to abort
                if self._quit_dock_event.is_set():
                    self.node.get_logger().info("Aborting docking")
                    return
                self.sleep_for(0.1)

            with self._lock:
                self.on_waypoint = self.dock_waypoint_index
                self.dock_waypoint_index = None
                self.docking_finished_callback()
                self.node.get_logger().info(f"Robot {self.name} has completed"
                                            " docking")

        self._dock_thread = threading.Thread(target=_dock)
        self._dock_thread.start()

    def get_position(self):
        ''' This helper function returns the live position of the robot in the
        RMF coordinate frame'''
        position = self.api.position(self.name)
        if position is not None:
            x, y = [position[0], position[1]]
            theta = position[2]
            # Wrap theta between [-pi, pi]. Else arrival estimate will
            # assume robot has to do full rotations and delay the schedule
            if theta > np.pi:
                theta = theta - (2 * np.pi)
            if theta < -np.pi:
                theta = (2 * np.pi) + theta
            return [x, y, theta]
        else:
            self.node.get_logger().error(
                "Unable to retrieve position from robot.")
            return self.position

    def get_battery_soc(self):
        battery_soc = self.api.battery_soc(self.name)
        if battery_soc is not None:
            return battery_soc
        else:
            self.node.get_logger().error(
                "Unable to retrieve battery data from robot.")
            return self.battery_soc

    def update(self):
        while rclpy.ok():
            self.position = self.get_position()
            self.battery_soc = self.get_battery_soc()
            if self.update_handle is not None:
                self.update_state()
            sleep_duration = float(1.0/self.update_frequency)
            self.sleep_for(sleep_duration)

    def update_state(self):
        self.update_handle.update_battery_soc(self.battery_soc)
        # Update position
        with self._lock:
            if (self.on_waypoint is not None):  # if robot is on a waypoint
                self.update_handle.update_current_waypoint(
                    self.on_waypoint, self.position[2])
            elif (self.on_lane is not None):  # if robot is on a lane
                # We only keep track of the forward lane of the robot.
                # However, when calling this update it is recommended to also
                # pass in the reverse lane so that the planner does not assume
                # the robot can only head forwards. This would be helpful when
                # the robot is still rotating on a waypoint.
                forward_lane = self.graph.get_lane(self.on_lane)
                entry_index = forward_lane.entry.waypoint_index
                exit_index = forward_lane.exit.waypoint_index
                reverse_lane = self.graph.lane_from(exit_index, entry_index)
                lane_indices = [self.on_lane]
                if reverse_lane is not None:  # Unidirectional graph
                    lane_indices.append(reverse_lane.index)
                self.update_handle.update_current_lanes(
                    self.position, lane_indices)
            elif (self.dock_waypoint_index is not None):
                self.update_handle.update_off_grid_position(
                    self.position, self.dock_waypoint_index)
            # if robot is performing an action
            elif (self.action_execution is not None):
                self.update_handle.update_off_grid_position(
                    self.position, self.action_waypoint_index)
            # if robot is merging into a waypoint
            elif (self.target_waypoint is not None and
                    self.target_waypoint.graph_index is not None):
                self.update_handle.update_off_grid_position(
                    self.position, self.target_waypoint.graph_index)
            else:  # if robot is lost
                self.update_handle.update_lost_position(
                    self.map_name, self.position)

    def get_current_lane(self):
        def projection(current_position,
                       target_position,
                       lane_entry,
                       lane_exit):
            px, py, _ = current_position
            p = np.array([px, py])
            t = np.array(target_position)
            entry = np.array(lane_entry)
            exit = np.array(lane_exit)
            return np.dot(p - t, exit - entry)

        if self.target_waypoint is None:
            return None
        approach_lanes = self.target_waypoint.approach_lanes
        # Spin on the spot
        if approach_lanes is None or len(approach_lanes) == 0:
            return None
        # Determine which lane the robot is currently on
        for lane_index in approach_lanes:
            lane = self.graph.get_lane(lane_index)
            p0 = self.graph.get_waypoint(lane.entry.waypoint_index).location
            p1 = self.graph.get_waypoint(lane.exit.waypoint_index).location
            p = self.position
            before_lane = projection(p, p0, p0, p1) < 0.0
            after_lane = projection(p, p1, p0, p1) >= 0.0
            if not before_lane and not after_lane:  # The robot is on this lane
                return lane_index
        return None

    def dist(self, A, B):
        ''' Euclidian distance between A(x,y) and B(x,y)'''
        assert(len(A) > 1)
        assert(len(B) > 1)
        return math.sqrt((A[0] - B[0])**2 + (A[1] - B[1])**2)

    def get_speed_limit(self, target_waypoint):
        approach_lane_limit = np.inf
        approach_lanes = target_waypoint.approach_lanes
        for lane_index in approach_lanes:
            lane = self.graph.get_lane(lane_index)
            lane_limit = lane.properties.speed_limit
            if lane_limit is not None:
                if lane_limit < approach_lane_limit:
                    approach_lane_limit = lane_limit
        return approach_lane_limit if approach_lane_limit != np.inf else 0.0

    def filter_waypoints(self, wps: list, threshold=1.0):
        ''' Return filtered PlanWaypoints'''

        assert(len(wps) > 0)
        first = None
        threshold = 0.5
        last_pose = copy.copy(self.position)
        waypoints = []
        for i in range(len(wps)):
            waypoints.append(PlanWaypoint(i, wps[i]))

        # We assume the first waypoint is safe for pruning if it is
        # within a threshold of the robot's current position
        first_position = waypoints[0].position
        if len(waypoints) > 2 and\
                self.dist(first_position, last_pose) < threshold:
            del waypoints[0]

        return (first, waypoints)

    def complete_robot_action(self):
        with self._lock:
            if self.action_execution is None:
                return
            self.action_execution.finished()
            self.action_execution = None
            self.node.get_logger().info(f"Robot {self.name} has completed the"
                                        f" action it was performing")

    def newly_closed_lanes(self, closed_lanes):
        need_to_replan = False
        current_lane = self.get_current_lane()

        if self.target_waypoint is not None and \
                self.target_waypoint.approach_lanes is not None:
            for lane_idx in self.target_waypoint.approach_lanes:
                if lane_idx in closed_lanes:
                    need_to_replan = True
                    # The robot is currently on a lane that has been closed.
                    # We take this to mean that the robot needs to reverse.
                    if lane_idx == current_lane:
                        lane = self.graph.get_lane(current_lane)

                        return_waypoint = lane.entry.waypoint_index
                        reverse_lane = \
                            self.graph.lane_from(lane.entry.waypoint_index,
                                                 lane.exit.waypoint_index)

                        with self._lock:
                            if reverse_lane:
                                # Update current lane to reverse back to
                                # start of the lane
                                self.on_lane = reverse_lane.index
                            else:
                                # Update current position and waypoint index
                                # to return to
                                self.target_waypoint = return_waypoint

        if not need_to_replan and self.target_waypoint is not None:
            # Check if the remainder of the current plan has been invalidated
            # by the lane closure
            for wp in self.remaining_waypoints:
                for lane in wp.approach_lanes:
                    if lane in closed_lanes:
                        need_to_replan = True
                        break
                if need_to_replan:
                    break

        if need_to_replan:
            self.update_handle.replan()

    def dock_summary_cb(self, msg):
        for fleet in msg.docks:
            if(fleet.fleet_name == self.fleet_name):
                for dock in fleet.params:
                    self.docks[dock.start] = dock.path

    def mode_request_cb(self, msg):
        if msg.fleet_name is None or msg.fleet_name != self.fleet_name or\
                msg.robot_name is None:
            return
        if msg.mode.mode == RobotState.IDLE:
            self.complete_robot_action()
