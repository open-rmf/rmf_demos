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
"""Dispatch an action."""

import json
import sys

from rmf_demos_tasks import task_api


class TaskRequester(task_api.TaskRequester):
    """Task requester."""

    request_prefix = 'custom_action'

    def add_arguments(self, parser):
        """Add action arguments."""
        parser.add_argument(
            '-s',
            '--starts',
            default=[],
            type=str,
            nargs='+',
            help='Action start waypoints',
        )
        parser.add_argument(
            '-a', '--action', required=True, type=str, help='action name'
        )
        parser.add_argument(
            '-ad',
            '--action_desc',
            required=False,
            default='{}',
            type=str,
            help='action description in json str',
        )
        parser.add_argument(
            '--use_tool_sink',
            action='store_true',
            help='Use tool sink during perform action, default: false',
        )

    def build_request(self, args):
        """Build an action request."""
        action = {
            'category': 'perform_action',
            'description': {
                'unix_millis_action_duration_estimate': 60000,
                'category': args.action,
                'description': json.loads(args.action_desc),
                'use_tool_sink': args.use_tool_sink,
            },
        }

        activities = []
        if not args.starts:
            activities.append(action)
        for start in args.starts:
            activities.append(
                {'category': 'go_to_place', 'description': start}
            )
            activities.append(action)

        return {
            'category': 'compose',
            'description': task_api.compose_sequence(args.action, activities),
        }


def main(argv=sys.argv):
    """Dispatch an action."""
    TaskRequester.run(argv)


if __name__ == '__main__':
    main(sys.argv)
