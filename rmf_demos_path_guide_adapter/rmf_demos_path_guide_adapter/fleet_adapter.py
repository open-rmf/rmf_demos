# Copyright 2026 Centre for Healthcare Assistive and Robotics Technology
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
Path Guide fleet adapter for the RMF Demos robots.

Based on rmf_demos_fleet_adapter/fleet_adapter.py, differing only in how
navigation work reaches the robot: EasyFullControl implements
`navigate(destination, execution)` and is handed one vertex at a time, while
this implements `follow_path(path)` and is handed the whole route at once.

**It measures no distances.** The fleet manager decides which waypoints were
reached; this adapter acknowledges up to the index it reports, one waypoint at
a time. See `acknowledge_reached_waypoints`.

Tracking a whole path rather than one execution forced three divergences from
the reference, each explained at its own definition: `stop` matches against
every waypoint in flight (`path_owns`), teleop ends through `end_teleop`, and a
dock is peeled off to `perform_docking`. `execute_action` additionally drops the
path in flight and has no `clean` case. The ROS 2 topic wiring is unchanged.

The `localize` callback is a temporary diagnostic -- see its docstring.
"""

import argparse
import asyncio
import faulthandler
import math
import sys
import threading
import time

import rclpy
from rclpy.duration import Duration
import rclpy.node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_system_default
from rclpy.qos import QoSDurabilityPolicy as Durability
from rclpy.qos import QoSHistoryPolicy as History
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy as Reliability
import rmf_adapter
from rmf_adapter import Adapter
import rmf_adapter.path_guide as rmf_pg
from rmf_fleet_msgs.msg import ClosedLanes
from rmf_fleet_msgs.msg import LaneRequest
from rmf_fleet_msgs.msg import ModeRequest
from rmf_fleet_msgs.msg import RobotMode
from rmf_fleet_msgs.msg import SpeedLimitRequest
import yaml

from .robot_client_api import RobotAPI
from .robot_client_api import RobotAPIResult
from .robot_client_api import RobotUpdateData


# -----------------------------------------------------------------------------
# Main
# -----------------------------------------------------------------------------
def main(argv=sys.argv):
    faulthandler.enable()
    # Init rclpy and adapter
    rclpy.init(args=argv)
    rmf_adapter.init_rclcpp()
    args_without_ros = rclpy.utilities.remove_ros_args(argv)

    parser = argparse.ArgumentParser(
        prog='path_guide_fleet_adapter',
        description='Configure and spin up the path guide fleet adapter',
    )
    parser.add_argument(
        '-c',
        '--config_file',
        type=str,
        required=True,
        help='Path to the config.yaml file',
    )
    parser.add_argument(
        '-n',
        '--nav_graph',
        type=str,
        required=True,
        help='Path to the nav_graph for this fleet adapter',
    )
    parser.add_argument(
        '-sim',
        '--use_sim_time',
        action='store_true',
        help='Use sim time, default: false',
    )
    args = parser.parse_args(args_without_ros[1:])
    print('Starting path guide fleet adapter...')

    config_path = args.config_file
    nav_graph_path = args.nav_graph

    fleet_config = rmf_pg.FleetConfiguration.from_config_files(
        config_path, nav_graph_path
    )
    assert fleet_config, f'Failed to parse config file [{config_path}]'

    # Parse the yaml in Python to get the fleet_manager info
    with open(config_path, 'r') as f:
        config_yaml = yaml.safe_load(f)

    # ROS 2 node for the command handle
    fleet_name = fleet_config.fleet_name
    node = rclpy.node.Node(f'{fleet_name}_path_guide_command_handle')
    adapter = Adapter.make(f'{fleet_name}_path_guide_fleet_adapter')
    assert adapter, (
        'Unable to initialize fleet adapter. '
        'Please ensure RMF Schedule Node is running'
    )

    # Enable sim time for testing offline
    if args.use_sim_time:
        param = Parameter('use_sim_time', Parameter.Type.BOOL, True)
        node.set_parameters([param])
        adapter.node.use_sim_time()

    adapter.start()
    time.sleep(1.0)

    node.declare_parameter('server_uri', '')
    server_uri = (
        node.get_parameter('server_uri').get_parameter_value().string_value
    )
    if server_uri == '':
        server_uri = None

    fleet_config.server_uri = server_uri
    fleet_handle = adapter.add_path_guide_fleet(fleet_config)
    fleet_handle.more().set_planner_cache_reset_size(2500)

    # Initialize robot API for this fleet
    fleet_mgr_yaml = config_yaml['fleet_manager']
    update_period = 1.0 / fleet_mgr_yaml.get(
        'robot_state_update_frequency', 10.0
    )
    api_prefix = (
        'http://' + fleet_mgr_yaml['ip'] + ':' + str(fleet_mgr_yaml['port'])
    )
    api = RobotAPI(
        api_prefix, fleet_mgr_yaml['user'], fleet_mgr_yaml['password']
    )

    robots = {}
    for robot_name in fleet_config.known_robots:
        robot_config = fleet_config.get_known_robot_configuration(robot_name)
        robots[robot_name] = RobotAdapter(
            robot_name, robot_config, node, api, fleet_handle
        )

    def update_loop():
        reassign_task_interval = config_yaml['rmf_fleet'].get(
            'reassign_task_interval', 60)  # seconds
        last_task_replan = node.get_clock().now()
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

            interval_sec = (now.nanoseconds -
                            last_task_replan.nanoseconds) / 1e9
            if interval_sec > reassign_task_interval:
                fleet_handle.more().reassign_dispatched_tasks()
                last_task_replan = now

            next_wakeup = now + Duration(nanoseconds=update_period * 1e9)
            while node.get_clock().now() < next_wakeup:
                time.sleep(0.001)

    update_thread = threading.Thread(target=update_loop, args=())
    update_thread.start()

    # Connect to the extra ROS2 topics that are relevant for the adapter
    connections = ros_connections(node, robots, fleet_handle)
    connections  # Avoid unused variable warning

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

    # Ceiling on the arrival radius at a waypoint inside a lift cabin, in
    # metres. Only ever tightens.
    #
    # Crediting that waypoint is what lets the doors close, so "arrived" there
    # must mean the whole robot is in the cabin. Credited on a radius sized for
    # open corridors, a 0.9 m-radius robot in this cabin had 0.778 m of itself
    # still outside when the doors were told to close, and the lift travelled
    # with it jammed in the doorway. See CLAUDE.md.
    #
    # RE-DERIVE THIS FOR YOUR FLEET. It is not a universal constant, and it
    # does NOT track profile.footprint in the config -- the two can disagree.
    # A circular footprint of radius r is entirely inside when its centre is
    # within (half_extent - r). The shipped 0.45 is a 0.9 m-radius robot in a
    # 2.7 m square cabin:
    #     2.7 / 2 - 0.9 = 0.45
    # A smaller robot in the same cabin gets a larger value, and leaving 0.45
    # in place merely costs a stricter test than needed; a LARGER robot, or a
    # tighter cabin, makes 0.45 unsafe.
    #
    # A constant, NOT derived from destination.inside_lift at runtime. That
    # geometry is RMF canonical while our position and the published bound are
    # robot coordinates; mixing them is silently wrong on any fleet setting
    # `transforms` and silently right on every rmf_demos config. If this fleet
    # ever sets a scale, scale this too.
    #
    # Caveat: a robot whose controller stops further out than this never gets
    # the waypoint credited and the path strands in the cabin -- the safer of
    # the two failures, but re-derive rather than widen blindly.
    LIFT_ARRIVAL_RADIUS = 0.45

    def __init__(
        self, name: str, configuration, node, api: RobotAPI, fleet_handle
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

        # Path Guide state, read and written as a set from two threads:
        # `follow_path` and `stop` run on PathGuide's worker, `update` on the
        # executor thread. `waypoints` and `current_index` are a matched pair,
        # so a path swapped partway through a read pairs a new, possibly
        # shorter list with an index from the old one. The reference adapter
        # needs no lock because it only ever swaps `self.execution`, a single
        # reference, which is atomic in CPython.
        #
        # Nothing blocking runs under this lock: finished() and update() only
        # post to the C++ worker and return.
        # Re-entrant because acknowledge_reached_waypoints calls other guarded
        # methods.
        self.path_lock = threading.RLock()
        self.path = None
        self.waypoints = []
        self.current_index = 0

    def update(self, state, data: RobotUpdateData):
        activity_identifier = None

        # Acknowledge first, then report the identifier of whatever waypoint we
        # are now approaching. Both finished() and update() are dispatched onto
        # the same adapter worker, in call order (PathGuide.cpp: finish() and
        # PathRobotUpdateHandle::update both go through worker.schedule), so by
        # the time the adapter handles this update it has already advanced past
        # the waypoints we just acknowledged and the identifier below is the one
        # it expects. Do not reorder these two steps.
        #
        # Both steps read the same path, so they are held under one lock
        # acquisition: releasing between them would let follow_path install a
        # new path and leave us reporting an identifier from neither.
        with self.path_lock:
            if self.path is not None:
                self.acknowledge_reached_waypoints(data)

            if self.path is not None:
                # This identifier tells RMF which path index the robot is
                # approaching, and it may NEVER decrease for a path: RMF turns
                # it into an unclamped itinerary delay, so going backwards can
                # deadlock other robots. PathGuide enforces it in C++.
                # `current_index` only ever increases, so this is safe.
                # Reporting the *nearest* waypoint is the obvious thing to
                # write and backtracks the moment the robot reverses. Don't.
                activity_identifier = self.current_waypoint().execution.identifier

        # Snapshot before testing it: follow_path nulls `self.execution` on the
        # worker thread, so checking the attribute and then dereferencing it
        # would be a use-after-null on this thread. A local costs nothing and
        # is enough -- a superseded execution is inert, so acting on one we
        # sampled a moment ago is harmless.
        execution = self.execution
        if self.path is None and execution is not None:
            # A non-path command -- an action such as teleop or a delivery
            # pickup, or a dock -- which completes via the fleet manager's
            # command id rather than through a reached waypoint index.
            if data.is_command_completed(self.cmd_id):
                execution.finished()
                # Only clear if it is still the one we just finished. A path or
                # action arriving in between installs its own, and nulling that
                # would strand it exactly the way this branch exists to prevent.
                if self.execution is execution:
                    self.execution = None
                    # Not a bare `self.teleoperation = None`: if that command
                    # WAS the teleop, the manager must still be told, or it
                    # stays in perform_action_mode. That bare clear is the
                    # reference adapter's bug, inherited. Free otherwise --
                    # end_teleop returns at once when no teleop is tracked.
                    # retry=True because nothing is dispatched after this.
                    self.end_teleop('the command completed', retry=True)
            else:
                activity_identifier = execution.identifier

        # Sampled, like `execution` above: end_teleop nulls this from
        # follow_path on the worker thread.
        teleoperation = self.teleoperation
        if teleoperation is not None:
            teleoperation.update(data)

        self.update_handle.update(state, activity_identifier)

    def make_callbacks(self):
        callbacks = rmf_pg.RobotCallbacks(
            lambda path: self.follow_path(path),
            lambda activity: self.stop(activity),
            lambda category, description, execution: self.execute_action(
                category, description, execution
            ),
        )
        # DIAGNOSTIC, 2026-08-28. See RobotAdapter.localize.
        callbacks.localize = (
            lambda destination, execution: self.localize(
                destination, execution
            )
        )
        return callbacks

    def localize(self, destination, execution):
        """
        Report that a localization request arrived, then complete it.

        **DIAGNOSTIC, not a real implementation**, and scheduled for removal. It
        exists to show that a PathGuide fleet receives the post-lift
        relocalization request at all; until 2026-08-28 it did not, and no demo
        run could reveal that because neither adapter registered a callback.
        `rmf_demos_fleet_adapter` carries the same one so the two flavours can
        be compared on one lift ride -- grep both logs for `LOCALIZE FIRED`.

        **Registering a callback is not behaviour-neutral.** Without one,
        `RobotContext::localize` returns false and both `RequestLift` and
        `follow_new_path`'s map-mismatch branch proceed immediately. With one
        they defer until `finished()`, behind a 300 s watchdog. So forgetting to
        call it is worse than registering nothing: 300 s of stall per site, a
        timeout logged, then recovery. A real integrator drives its
        relocalization routine here and finishes when the robot has actually
        re-localized -- from any thread, since `finished()` schedules onto the
        RMF worker rather than running inline.
        """
        position = destination.position
        self.node.get_logger().info(
            f'[{self.name}] LOCALIZE FIRED [PathGuide]: asked to localize on '
            f'map [{destination.map}] at '
            f'({position[0]:.2f}, {position[1]:.2f}, {position[2]:.2f})'
        )
        execution.finished()

    def current_waypoint(self):
        """
        Return the waypoint we are still trying to reach.

        Call with `path_lock` held and only while `self.path` is not None. Under
        those conditions `current_index` is always a valid index into
        `waypoints`, because acknowledge_through clears the path in the same
        critical section that carries the index past the end.
        """
        with self.path_lock:
            return self.waypoints[self.current_index]

    def set_path(self, path, waypoints=None):
        """
        Adopt a newly dispatched path.

        The waypoint list is cached here because `path.waypoints` crosses the
        pybind boundary and deep-copies every waypoint on each access. The
        copies share their identifiers, so caching changes nothing semantically.

        `waypoints` lets a caller that already crossed that boundary hand the
        list in, keeping the copy out from under `path_lock`. Takes the lock
        itself; called from the worker thread.
        """
        if waypoints is None:
            # Cross the pybind boundary before taking the lock: this rebuilds
            # the list and deep-copies every waypoint, and there is no reason
            # to make the update thread wait for it.
            waypoints = list(path.waypoints)
        with self.path_lock:
            self.path = path
            self.waypoints = waypoints
            self.current_index = 0

    def clear_path(self):
        """Forget the path in flight, whether it finished or was abandoned."""
        with self.path_lock:
            self.path = None
            self.waypoints = []
            self.current_index = 0

    def path_owns(self, activity):
        """
        Whether `activity` belongs to any waypoint of the path in flight.

        Not only the waypoint we track. Our `current_index` advances as we
        acknowledge; the adapter's advances when those acknowledgements reach
        its worker, so a stop queued ahead of them names an earlier waypoint. A
        single-index comparison would miss it, leaving us driving a path the
        adapter has abandoned.
        """
        with self.path_lock:
            return any(
                wp.execution.identifier.is_same(activity)
                for wp in self.waypoints
            )

    def follow_path(self, path):
        """
        Send the robot its entire route in one request.

        This replaces the reference adapter's navigate(). Any previously
        dispatched path is superseded, and PathGuide invalidates that path's
        CommandExecutions as it installs this one, so dropping our reference is
        enough - late acknowledgements against the old handles are inert.
        """
        self.cmd_id += 1
        self.execution = None
        self.end_teleop('superseded by a new path')

        # Cross the pybind boundary BEFORE installing: `path.waypoints`
        # rebuilds the list and deep-copies every waypoint, and doing that
        # under `path_lock` would stall every update tick for the length of
        # the route.
        #
        # Then hand that same list to set_path, so `waypoints` IS the list
        # installed and nothing is read back. Reading `self.waypoints`
        # afterwards was the 2026-08-21 race: a concurrent completion calls
        # clear_path and the read-back comes back empty, dispatching an empty
        # route. Not reading it back is what closes that, NOT a lock.
        #
        # set_path takes `path_lock` itself, so no acquisition here -- and
        # none should span the dispatch below, which joins a 5 s HTTP call.
        waypoints = list(path.waypoints)
        self.set_path(path, waypoints)

        maps = []
        for wp in waypoints:
            if not maps or maps[-1] != wp.destination.map:
                maps.append(wp.destination.map)
        self.node.get_logger().info(
            f'Commanding [{self.name}] to follow a path of '
            f'{len(waypoints)} waypoint(s) across {" -> ".join(maps)}: '
            f'cmd_id {self.cmd_id}, plan_id {path.plan_id}'
        )

        # PathGuide delivers a dock maneuver as a single-waypoint path with the
        # dock name set, so that the integrator only has one callback shape to
        # implement. The fleet manager still wants it as an activity rather than
        # a path, so peel it off here.
        if len(waypoints) == 1 and waypoints[0].destination.dock is not None:
            # Treat it as a non-path command, exactly as the reference does:
            # hold the execution and let the manager's command id complete it
            # in update().
            #
            # A dock cannot complete the way a route does -- start-activity
            # sets no `path_id`, so `reached_index_for` returns None forever.
            # Leaving the path installed hangs it, and also blocks update()'s
            # completion branch, which requires `self.path is None`. Nothing is
            # lost by dropping it: PathGuide reports no path index for a dock.
            dock_waypoint = waypoints[0]
            self.execution = dock_waypoint.execution
            self.clear_path()
            self.attempt_cmd_until_success(
                cmd=self.perform_docking, args=(dock_waypoint,)
            )
            return

        self.attempt_cmd_until_success(
            cmd=self.api.follow_path,
            args=(self.name, self.cmd_id, self.as_request(waypoints)),
        )

    def as_request(self, waypoints):
        """
        Render the path for the fleet manager.

        `arrival_radius` is `arrival_radius_for(wp)` -- our decision, not
        anything PathGuide published. The manager measures arrival against
        whatever we send, so the lift cap has to travel with the waypoint.
        Sending `wp.merge_radius` here would send the *bound* instead of our
        choice, putting the door-closing bug straight back.
        """
        request = []
        for wp in waypoints:
            destination = wp.destination
            request.append(
                {
                    'map_name': destination.map,
                    'x': destination.position[0],
                    'y': destination.position[1],
                    'yaw': destination.position[2],
                    'speed_limit': destination.speed_limit,
                    'arrival_radius': self.arrival_radius_for(wp),
                }
            )
        return request

    def acknowledge_reached_waypoints(self, data: RobotUpdateData):
        """
        Acknowledge every waypoint the fleet manager says the robot reached.

        **This adapter measures no distances.** The manager decides what was
        reached and reports the furthest as `reached_waypoint_index`; see
        `FleetManager.advance_reached_index`.

        What stays here is ordering. PathGuide accepts `finished()` only for the
        waypoint it currently tracks, so a reported index of 7 is walked up one
        at a time -- the manager reports a level, PathGuide's contract is in
        edges, and `acknowledge_through` is that translation.

        Takes `path_lock` itself so it is safe to call alone; `update` already
        holds it across this and the identifier read that follows.
        """
        with self.path_lock:
            self._acknowledge_reached_waypoints(data)

    def arrival_radius_for(self, wp):
        """
        Choose how close the robot must be before we credit `wp`.

        **This decision is entirely ours.** PathGuide publishes no arrival
        radius; `wp.merge_radius` is a *tolerance* -- how far off its lane RMF
        can still localize the robot here -- and therefore an upper bound on how
        loose we may be, not a value to use as-is. Credit a path's final
        waypoint beyond it and the robot stops somewhere RMF cannot merge it
        back from.

        Starting at that bound is the loosest safe choice, which is what a
        rounded corner needs. Inside a lift we tighten to
        `LIFT_ARRIVAL_RADIUS`, because crediting the waypoint is what lets the
        doors close and "arrived" there must mean the whole robot is in the
        cabin.

        `min`, never `max`: this may only tighten. All in robot coordinates; see
        `LIFT_ARRIVAL_RADIUS` for why the cabin geometry is not consulted.
        """
        if wp.destination.inside_lift is None:
            return wp.merge_radius

        return min(wp.merge_radius, self.LIFT_ARRIVAL_RADIUS)

    def _acknowledge_reached_waypoints(self, data: RobotUpdateData):
        reached = data.reached_index_for(self.cmd_id)
        if reached is None:
            # The manager has no opinion yet, or is still measuring the path
            # this one replaced. Nothing to credit either way.
            return

        if reached < self.current_index:
            # Everything it reports has already been acknowledged. Normal
            # while our acknowledgements are still draining to the C++ worker.
            return

        if reached >= len(self.waypoints):
            # Only reachable if the manager is measuring a different path under
            # the same id, which would mean the ids have gone wrong somewhere.
            # Refusing is safer than crediting waypoints that do not exist.
            self.node.get_logger().error(
                f'[{self.name}] fleet manager reports waypoint {reached} of '
                f'path {self.cmd_id}, but that path has only '
                f'{len(self.waypoints)} waypoint(s). Ignoring.'
            )
            return

        self.acknowledge_through(reached)

    def acknowledge_through(self, reached):
        """
        Acknowledge `current_index` up to and including `reached`.

        Call with `path_lock` held. Every caller reaches it through
        acknowledge_reached_waypoints, which takes it.
        """
        waypoints = self.waypoints
        for i in range(self.current_index, reached + 1):
            destination = waypoints[i].destination
            self.node.get_logger().info(
                f'[{self.name}] reached waypoint {i}/'
                f'{len(waypoints) - 1} '
                f'({destination.position[0]:.2f}, '
                f'{destination.position[1]:.2f}) on [{destination.map}]'
            )
            waypoints[i].execution.finished()

        self.current_index = reached + 1
        if self.current_index >= len(waypoints):
            # The whole path is done. The adapter will send a new one when it
            # has more work for this robot.
            self.clear_path()

    def stop(self, activity):
        # Test and act atomically: this runs on the adapter's worker thread
        # while update() may be acknowledging on the update thread, and a path
        # that is cleared between the check and the clear would leave us
        # driving a route the adapter has abandoned.
        with self.path_lock:
            owned = self.path is not None and self.path_owns(activity)
            if owned:
                self.clear_path()

        if owned:
            self.issue_stop()
            return

        # Sampled: update() nulls `execution` from the update thread, and an
        # AttributeError here escapes back into PathGuide's rxcpp worker --
        # and the stop would never be issued.
        execution = self.execution
        if execution is not None and execution.identifier.is_same(activity):
            if self.execution is execution:
                self.execution = None
            # If that activity was the teleop, this is what ends it -- update()
            # cannot, since its clearing branch needs `execution` non-None.
            self.end_teleop('the activity was stopped')
            self.issue_stop()

    def issue_stop(self):
        running_cmd_id = self.cmd_id
        self.cmd_id += 1
        stop_cmd_id = self.cmd_id
        self.attempt_cmd_until_success(
            cmd=self.api.stop,
            args=(self.name, running_cmd_id, stop_cmd_id)
        )

    def execute_action(self, category: str, description: dict, execution):
        self.cmd_id += 1
        self.execution = execution
        # An action is not a path, so stop tracking whichever path we were on.
        self.clear_path()

        if category != 'teleop':
            # A different action supersedes any teleop in flight. Not for
            # 'teleop' itself: that replaces the Teleoperation and is about to
            # toggle the mode back on, so ending it here would send False then
            # True for no reason.
            self.end_teleop(f'superseded by action [{category}]')

        match category:
            case 'teleop':
                self.teleoperation = Teleoperation(execution)
                self.attempt_cmd_until_success(
                    cmd=self.api.toggle_teleop, args=(self.name, True)
                )
            case 'delivery_pickup':
                self.attempt_cmd_until_success(
                    cmd=self.api.toggle_attach, args=(
                        self.name, True, self.cmd_id)
                )
            case 'delivery_dropoff':
                self.attempt_cmd_until_success(
                    cmd=self.api.toggle_attach, args=(
                        self.name, False, self.cmd_id)
                )

    def end_teleop(self, reason, retry=False):
        """
        End a teleop session, whatever ended it.

        **Every path into "teleop is over" must come through here** -- a new
        path, a different action, a stop, the operator's ModeRequest, and the
        session's own command completing in `update`. Each must drop the
        `Teleoperation` *and* tell the fleet manager. Miss the second and the
        manager stays in `perform_action_mode`, which suppresses its
        stale-task-id republish: a dropped `PathRequest` is never resent and the
        robot silently stops.

        `retry` is forced by what happens next:

        * `retry=True` from `finish_action` and `update`, where nothing is
          dispatched afterwards, so the retry loop survives.
        * `retry=False` elsewhere, because the caller dispatches immediately and
          would cancel its own retry. One attempt on a thread instead.

        Either way the in-flight `toggle_teleop(True)` is joined first:
        cancelling only sets an event and cannot abort a POST already in flight,
        so a stale True landing after our False would latch
        `perform_action_mode` back on.

        A failed toggle has no in-adapter recovery in the `retry=False` case.
        """
        if self.teleoperation is None:
            return

        self.teleoperation = None
        self.node.get_logger().info(
            f'[{self.name}] teleop ended: {reason}'
        )

        if retry:
            self.attempt_cmd_until_success(
                cmd=self.api.toggle_teleop, args=(self.name, False)
            )
            return

        # Join any in-flight toggle_teleop(True) before sending False.
        self.cancel_cmd_attempt()
        threading.Thread(
            target=self.api.toggle_teleop, args=(self.name, False)
        ).start()

    def finish_action(self):
        # This is triggered by a ModeRequest callback which allows human
        # operators to manually change the operational mode of the robot. This
        # is typically used to indicate when teleoperation has finished.
        # Sampled before use: follow_path nulls `execution` from PathGuide's
        # worker thread, and this runs on a rclpy subscription callback, where
        # an AttributeError would escape into spin() and take the node down.
        execution = self.execution
        if execution is None:
            return

        execution.finished()
        # Only clear what we finished -- a path or action arriving in between
        # installs its own, and nulling that would strand it.
        if self.execution is execution:
            self.execution = None

        self.end_teleop('the operator ended it', retry=True)

    def perform_docking(self, waypoint):
        destination = waypoint.destination
        match self.api.start_activity(
            self.name, self.cmd_id, 'dock', destination.dock
        ):
            case (RobotAPIResult.SUCCESS, path):
                self.override = waypoint.execution.override_schedule(
                    path['map_name'], path['path']
                )
                return True
            case RobotAPIResult.RETRY:
                return False
            case RobotAPIResult.IMPOSSIBLE:
                # If the fleet manager does not know this dock name, then treat
                # it as a regular single-waypoint path.
                return self.api.follow_path(
                    self.name, self.cmd_id, self.as_request([waypoint])
                )

    def attempt_cmd_until_success(self, cmd, args):
        self.cancel_cmd_attempt()

        def loop():
            while not cmd(*args):
                self.node.get_logger().warn(
                    f'Failed to contact fleet manager for robot {self.name}'
                )
                if self.cancel_cmd_event.wait(1.0):
                    break

        self.issue_cmd_thread = threading.Thread(target=loop, args=())
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
            dist = math.sqrt(dx * dx + dy * dy)
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

    state = rmf_pg.RobotState(data.map, data.position, data.battery_soc)

    if robot.update_handle is None:
        robot.update_handle = robot.fleet_handle.add_robot(
            robot.name, state, robot.configuration, robot.make_callbacks()
        )
        return

    robot.update(state, data)


def ros_connections(node, robots, fleet_handle):
    fleet_name = fleet_handle.more().fleet_name

    transient_qos = QoSProfile(
        history=History.KEEP_LAST,
        depth=1,
        reliability=Reliability.RELIABLE,
        durability=Durability.TRANSIENT_LOCAL,
    )

    closed_lanes_pub = node.create_publisher(
        ClosedLanes, 'closed_lanes', qos_profile=transient_qos
    )

    closed_lanes = set()

    def lane_request_cb(msg):
        if msg.fleet_name and msg.fleet_name != fleet_name:
            print(f'Ignoring lane request for fleet [{msg.fleet_name}]')
            return

        if msg.open_lanes:
            print(f'Opening lanes: {msg.open_lanes}')

        if msg.close_lanes:
            print(f'Closing lanes: {msg.close_lanes}')

        fleet_handle.more().open_lanes(msg.open_lanes)
        fleet_handle.more().close_lanes(msg.close_lanes)

        for lane_idx in msg.close_lanes:
            closed_lanes.add(lane_idx)

        for lane_idx in msg.open_lanes:
            if lane_idx in closed_lanes:
                closed_lanes.remove(lane_idx)

        state_msg = ClosedLanes()
        state_msg.fleet_name = fleet_name
        state_msg.closed_lanes = list(closed_lanes)
        closed_lanes_pub.publish(state_msg)

    def speed_limit_request_cb(msg):
        if msg.fleet_name is None or msg.fleet_name != fleet_name:
            return

        requests = []
        for limit in msg.speed_limits:
            request = rmf_adapter.fleet_update_handle.SpeedLimitRequest(
                limit.lane_index, limit.speed_limit)
            requests.append(request)
        fleet_handle.more().limit_lane_speeds(requests)
        fleet_handle.more().remove_speed_limits(msg.remove_limits)

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

    lane_request_sub = node.create_subscription(
        LaneRequest,
        'lane_closure_requests',
        lane_request_cb,
        qos_profile=qos_profile_system_default,
    )

    speed_limit_request_sub = node.create_subscription(
        SpeedLimitRequest,
        'speed_limit_requests',
        speed_limit_request_cb,
        qos_profile=qos_profile_system_default,
    )

    action_execution_notice_sub = node.create_subscription(
        ModeRequest,
        'action_execution_notice',
        mode_request_cb,
        qos_profile=qos_profile_system_default,
    )

    return [
        lane_request_sub,
        speed_limit_request_sub,
        action_execution_notice_sub,
    ]


if __name__ == '__main__':
    main(sys.argv)
