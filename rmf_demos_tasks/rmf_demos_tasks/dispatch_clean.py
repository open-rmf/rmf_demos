#!/usr/bin/env python3

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
"""Dispatch a cleaning."""

import sys

from rmf_demos_tasks import task_api


class TaskRequester(task_api.TaskRequester):
    """Task requester."""

    request_prefix = 'clean'

    def add_arguments(self, parser):
        """Add cleaning arguments."""
        parser.add_argument(
            '-cs',
            '--clean_start',
            required=True,
            type=str,
            help='Cleaning zone',
        )

    def build_request(self, args):
        """Build a cleaning request."""
        activities = [
            # Send robot to start waypoint first
            {'category': 'go_to_place', 'description': args.clean_start},
            # Send a perform action request for robot to clean area
            {
                'category': 'perform_action',
                'description': {
                    'unix_millis_action_duration_estimate': 60000,
                    'category': 'clean',
                    'expected_finish_location': args.clean_start,
                    'description': {'zone': args.clean_start},
                    'use_tool_sink': True,
                },
            },
        ]
        return {
            'category': 'compose',
            'description': task_api.compose_sequence('clean', activities),
        }


def main(argv=sys.argv):
    """Dispatch a cleaning."""
    TaskRequester.run(argv)


if __name__ == '__main__':
    main(sys.argv)
