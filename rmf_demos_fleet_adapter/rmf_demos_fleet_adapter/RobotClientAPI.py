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


'''
    The RobotAPI class is a wrapper for API calls to the robot. Here users
    are expected to fill up the implementations of functions which will be used
    by the RobotCommandHandle. For example, if your robot has a REST API, you
    will need to make http request calls to the appropriate endpoints within
    these functions.
'''

import requests
import enum
from urllib.error import HTTPError


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
        ''' Return True if connection to the robot API server is successful'''
        if self.data() is None:
            return False
        return True

    def navigate(
        self,
        robot_name: str,
        cmd_id: int,
        pose,
        map_name: str,
        speed_limit=0.0
    ):
        ''' Request the robot to navigate to pose:[x,y,theta] where x, y and
            and theta are in the robot's coordinate convention. This function
            should return True if the robot has accepted the request,
            else False'''
        assert(len(pose) > 2)
        url = self.prefix +\
            f'/open-rmf/rmf_demos_fm/navigate?robot_name={robot_name}' \
            f'&cmd_id={cmd_id}'
        data = {}  # data fields: task, map_name, destination{}, data{}
        data['map_name'] = map_name
        data['destination'] = {'x': pose[0], 'y': pose[1], 'yaw': pose[2]}
        data['speed_limit'] = speed_limit
        try:
            response = requests.post(url, timeout=self.timeout, json=data)
            response.raise_for_status()
            if self.debug:
                print(f'Response: {response.json()}')
            return response.json()['success']
        except HTTPError as http_err:
            print(f'HTTP error for {robot_name} in navigate: {http_err}')
        except Exception as err:
            print(f'Other error for {robot_name} in navigate: {err}')
        return False

    def start_activity(
        self,
        robot_name: str,
        cmd_id: int,
        activity: str,
        label: str
    ):
        ''' Request the robot to begin a process. This is specific to the robot
            and the use case. For example, load/unload a cart for Deliverybot
            or begin cleaning a zone for a cleaning robot.'''
        url = (
            self.prefix +
            f"/open-rmf/rmf_demos_fm/start_activity?robot_name={robot_name}"
            f"&cmd_id={cmd_id}"
        )
        # data fields: task, map_name, destination{}, data{}
        data = {'activity': activity, 'label': label}
        try:
            response = requests.post(url, timeout=self.timeout, json=data)
            response.raise_for_status()
            if self.debug:
                print(f'Response: {response.json()}')

            if response.json()['success']:
                return (
                    RobotAPIResult.SUCCESS,
                    response.json()['data']['path']
                )

            # If we get a response with success=False, then
            return RobotAPIResult.IMPOSSIBLE
        except HTTPError as http_err:
            print(f'HTTP error for {robot_name} in start_activity: {http_err}')
        except Exception as err:
            print(f'Other error {robot_name} in start_activity: {err}')
        return RobotAPIResult.RETRY

    def stop(self, robot_name: str, cmd_id: int):
        ''' Command the robot to stop.
            Return True if robot has successfully stopped. Else False'''
        url = self.prefix +\
            f'/open-rmf/rmf_demos_fm/stop_robot?robot_name={robot_name}' \
            f'&cmd_id={cmd_id}'
        try:
            response = requests.get(url, self.timeout)
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
        '''Request to toggle the robot's mode_teleop parameter.
           Return True if the toggle request is successful'''
        url = self.prefix +\
            f"/open-rmf/rmf_demos_fm/toggle_teleop?robot_name={robot_name}"
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

    def get_data(self, robot_name: str | None = None):
        """
        Return a RobotUpdateData for one robot if a name is given. Otherwise
        return a list of RobotUpdateData for all robots.
        """
        if robot_name is None:
            url = self.prefix + f'/open-rmf/rmf_demos_fm/status'
        else:
            url = self.prefix +\
                f'/open-rmf/rmf_demos_fm/status?robot_name={robot_name}'
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
    """Update data for a single robot"""
    def __init__(self, data):
        self.robot_name = data['robot_name']
        position = data['position']
        x = position['x']
        y = position['y']
        yaw = position['yaw']
        self.position = [x, y, yaw]
        self.map = data['map_name']
        self.battery_soc = data['battery']/100.0
        self.requires_replan = data.get('replan', False)
        self.last_request_completed = data['last_completed_request']

    def is_command_completed(self, cmd_id):
        return self.last_request_completed == cmd_id
