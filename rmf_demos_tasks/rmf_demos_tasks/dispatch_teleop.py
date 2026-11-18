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
"""Dispatch a teleop task."""

import sys

from rmf_demos_tasks import task_api


class TaskRequester(task_api.TaskRequester):
    """Task requester."""

    request_prefix = 'teleop'

    def add_arguments(self, parser):
        """Add teleop arguments."""
        parser.add_argument(
            '-s', '--start', required=True, type=str, help='Start waypoint'
        )

    def build_request(self, args):
        """Build a teleop request."""
        activities = [
            {'category': 'go_to_place', 'description': args.start},
            {
                'category': 'perform_action',
                'description': {
                    'unix_millis_action_duration_estimate': 60000,
                    'category': 'teleop',
                    'description': {},
                },
            },
        ]
        return {
            'category': 'compose',
            'description': task_api.compose_sequence('teleop', activities),
        }


def main(argv=sys.argv):
    """Dispatch a teleop task."""
    TaskRequester.run(argv)


if __name__ == '__main__':
    main(sys.argv)
