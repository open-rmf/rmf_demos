#!/usr/bin/env python3

# Copyright 2022 Open Source Robotics Foundation, Inc.
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

from __future__ import annotations

import requests
from yaml import YAMLObject
from typing import Optional

from rmf_door_msgs.msg import DoorMode
from rclpy.impl.rcutils_logger import RcutilsLogger


"""
    The DoorAPI class is a wrapper for API calls to the door.

    Here users are expected to fill up the implementations of functions which
    will be used by the DoorAdapter. For example, if your door has a REST API,
    you will need to make http request calls to the appropriate endpoints
    within these functions.
"""


class DoorAPI:
    # The constructor accepts a safe loaded YAMLObject, which should contain all
    # information that is required to run any of these API calls.
    def __init__(self, config: YAMLObject, logger: RcutilsLogger):
        self.config = config
        self.prefix = 'http://' + config['door_manager']['ip'] +\
            ':' + str(config['door_manager']['port'])
        self.logger = logger
        self.timeout = 1.0

    def door_mode(self, door_name) -> Optional[int]:
        """Returns the DoorMode or None if the query failed."""
        try:
            response = requests.get(self.prefix +
                                    f'/open-rmf/demo-door/door_state?door_name={door_name}',
                                    timeout=self.timeout)
        except Exception as err:
            self.logger.info(f'{err}')
            return None
        if response.status_code != 200 or response.json()['success'] is False:
            return None
        # In this example the door uses the same API as RMF, if it didn't
        # we would need to convert the result into a DoorMode here
        door_mode = response.json()['data']['current_mode']
        return door_mode

    def _command_door(self, door_name, requested_mode: int) -> bool:
        """Utility function to command doors.

        Returns True if the request was sent out successfully, False
        otherwise
        """
        try:
            data = {'requested_mode': requested_mode}
            response = requests.post(self.prefix +
                                     f'/open-rmf/demo-door/door_request?door_name={door_name}',
                                     timeout=self.timeout,
                                     json=data)
        except Exception as err:
            self.logger.info(f'{err}')
            return None
        if response.status_code != 200 or response.json()['success'] is False:
            return False
        return True

    def open_door(self, door_name):
        """Command the door to open.

        Returns True if the request was sent out successfully, False
        otherwise
        """
        return self._command_door(door_name, DoorMode.MODE_OPEN)

    def close_door(self, door_name):
        """Command the door to close.

        Returns True if the request was sent out successfully, False
        otherwise
        """
        return self._command_door(door_name, DoorMode.MODE_CLOSED)
