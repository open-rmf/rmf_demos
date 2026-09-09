# Copyright 2026 Open Source Robotics Foundation, Inc.
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
The RobotAPI class is a wrapper for API calls to the robot.

Based on rmf_demos_fleet_adapter/RobotClientAPI.py. The difference is that
navigate() is replaced by follow_path(), which sends the robot its entire route
in one request. Everything else - stop, activities, status polling - is
unchanged.

Here users are expected to fill up the implementations of functions which will
be used by the RobotCommandHandle. For example, if your robot has a REST API,
you will need to make http request calls to the appropriate endpoints within
these functions.
"""
import enum
from urllib.error import HTTPError

import requests


class RobotAPIResult(enum.IntEnum):
    SUCCESS = 0
    """The request was successful"""

    RETRY = 1
    """The client failed to connect but might succeed if you try again"""

    IMPOSSIBLE = 2
    """The client connected but something about the request is impossible"""


class RobotAPI:
    # The constructor below accepts parameters typically required to submit
    # http requests. Users should modify the constructor as per the
    # requirements of their robot's API
    def __init__(self, prefix: str, user: str, password: str):
        self.prefix = prefix
        self.user = user
        self.password = password
        self.timeout = 5.0
        self.debug = False

    def check_connection(self):
        """Return True if connection to the robot API server is successful."""
        if self.get_data() is None:
            return False
        return True

    def follow_path(self, robot_name: str, path_id: int, waypoints):
        """
        Request the robot to follow an entire path.

        Where waypoints is an ordered list of dicts, each with the keys
        map_name, x, y, yaw, speed_limit and arrival_radius. The coordinates
        are in the robot's coordinate convention.

        `arrival_radius` is the *integrator's* choice, not something PathGuide
        published: the adapter picks it per waypoint, bounded by that waypoint's
        merge_radius and tightened inside lift cabins. See
        RobotAdapter.arrival_radius_for.

        This replaces the per-waypoint navigate() call of the EasyFullControl
        adapter: the robot is given the whole route up front so a fleet manager
        can smooth or velocity-profile across it.

        This function should return True if the robot has accepted the request,
        else False.
        """
        assert len(waypoints) > 0
        url = (
            self.prefix
            + '/open-rmf/rmf_demos_pg/follow-path'
            f'?robot_name={robot_name}'
            f'&path_id={path_id}'
        )
        data = {'waypoints': waypoints}
        try:
            response = requests.post(url, timeout=self.timeout, json=data)
            response.raise_for_status()
            if self.debug:
                print(f'Response: {response.json()}')
            return response.json()['success']
        except HTTPError as http_err:
            print(f'HTTP error for {robot_name} in follow_path: {http_err}')
        except Exception as err:
            print(f'Other error for {robot_name} in follow_path: {err}')
        return False

    def start_activity(
        self, robot_name: str, cmd_id: int, activity: str, label: str
    ):
        """
        Request the robot to begin a process.

        This is specific to the robot and the use case. Here it is used for
        docking; the fleet manager returns the path the robot will take, which
        the caller pushes into the traffic schedule.
        """
        url = (
            self.prefix
            + '/open-rmf/rmf_demos_pg/start-activity'
            f'?robot_name={robot_name}'
            f'&cmd_id={cmd_id}'
        )
        data = {'activity': activity, 'label': label}
        try:
            response = requests.post(url, timeout=self.timeout, json=data)
            response.raise_for_status()
            if self.debug:
                print(f'Response: {response.json()}')

            if response.json()['success']:
                return (
                    RobotAPIResult.SUCCESS,
                    response.json()['data']['path'],
                )

            # If we get a response with success=False, then
            return RobotAPIResult.IMPOSSIBLE
        except HTTPError as http_err:
            print(f'HTTP error for {robot_name} in start_activity: {http_err}')
        except Exception as err:
            print(f'Other error {robot_name} in start_activity: {err}')
        return RobotAPIResult.RETRY

    def stop(self, robot_name: str, running_cmd_id: int, stop_cmd_id: int):
        """
        Command the robot to stop.

        Return True if robot has successfully stopped. Else False
        """
        url = (
            self.prefix
            + '/open-rmf/rmf_demos_pg/stop-robot'
            f'?robot_name={robot_name}'
            f'&running_cmd_id={running_cmd_id}'
            f'&stop_cmd_id={stop_cmd_id}'
        )
        try:
            response = requests.get(url, timeout=self.timeout)
            response.raise_for_status()
            if self.debug:
                print(f'Response: {response.json()}')
            return response.json()['success']
        except HTTPError as http_err:
            print(f'HTTP error for {robot_name} in stop: {http_err}')
        except Exception as err:
            print(f'Other error for {robot_name} in stop: {err}')
        return False

    def toggle_teleop(self, robot_name: str, toggle: bool):
        """
        Request to toggle the robot's mode_teleop parameter.

        Return True if the toggle request is successful
        """
        url = (
            self.prefix
            + '/open-rmf/rmf_demos_pg/toggle-teleop'
            f'?robot_name={robot_name}'
        )
        data = {'toggle': toggle}
        try:
            response = requests.post(url, timeout=self.timeout, json=data)
            response.raise_for_status()
            if self.debug:
                print(f'Response: {response.json()}')
            return response.json()['success']
        except HTTPError as http_err:
            print(f'HTTP error for {robot_name} in toggle_teleop: {http_err}')
        except Exception as err:
            print(f'Other error {robot_name} in toggle_teleop: {err}')
        return False

    def toggle_attach(self, robot_name: str, attach: bool, cmd_id: int):
        """
        Request to attach or detach robot to/from cart.

        Return True if the attach request is successful
        """
        url = (
            self.prefix
            + '/open-rmf/rmf_demos_pg/toggle-attach'
            f'?robot_name={robot_name}'
            f'&cmd_id={cmd_id}'
        )
        data = {'toggle': attach}
        try:
            response = requests.post(url, timeout=self.timeout, json=data)
            response.raise_for_status()
            if self.debug:
                print(f'Response: {response.json()}')
            return response.json()['success']
        except HTTPError as http_err:
            print(f'HTTP error for {robot_name} in toggle_attach: {http_err}')
        except Exception as err:
            print(f'Other error {robot_name} in toggle_attach: {err}')
        return False

    def get_data(self, robot_name: str | None = None):
        """
        Return a RobotUpdateData for one robot if a name is given.

        Otherwise return a list of RobotUpdateData for all robots.
        """
        if robot_name is None:
            url = self.prefix + '/open-rmf/rmf_demos_pg/status'
        else:
            url = (
                self.prefix
                + f'/open-rmf/rmf_demos_pg/status?robot_name={robot_name}'
            )
        try:
            response = requests.get(url, timeout=self.timeout)
            response.raise_for_status()
            if self.debug:
                print(f'Response: {response.json()}')
            if robot_name is not None:
                return RobotUpdateData(response.json()['data'])
            else:
                all_robots = []
                for robot in response.json()['all_robots']:
                    all_robots.append(RobotUpdateData(robot))
                return all_robots
        except HTTPError as http_err:
            print(f'HTTP error for {robot_name} in get_data: {http_err}')
        except Exception as err:
            print(f'Other error for {robot_name} in get_data: {err}')
        return None


class RobotUpdateData:
    """Update data for a single robot."""

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
        # The fleet manager's view of how far along its path the robot has got,
        # and which path that refers to. None / -1 when it has measured nothing
        # yet, and from an older manager that reports neither field.
        self.path_id = data.get('path_id')
        self.reached_waypoint_index = data.get('reached_waypoint_index', -1)

    def is_command_completed(self, cmd_id):
        return self.last_request_completed == cmd_id

    def reached_index_for(self, path_id):
        """
        Return the manager's progress through `path_id`, or None.

        None means it has no opinion: nothing measured yet, or it is tracking a
        different path than the caller. Guarding on the path id matters because
        an index is only meaningful against the path it was measured on --
        applying a stale one would credit waypoints the robot never visited.
        """
        if self.path_id is None or self.reached_waypoint_index is None:
            return None
        if str(self.path_id) != str(path_id):
            return None
        return self.reached_waypoint_index
