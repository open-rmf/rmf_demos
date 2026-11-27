#!/usr/bin/env python3

# Copyright 2023 Open Source Robotics Foundation, Inc.
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
"""Dispatch a cart delivery."""

import sys

from rmf_demos_tasks import task_api


class TaskRequester(task_api.TaskRequester):
    """Task requester."""

    request_prefix = 'delivery'

    def add_arguments(self, parser):
        """Add cart delivery arguments."""
        parser.add_argument('-p', '--pickup', required=True,
                            type=str,
                            help='Pickup waypoint name')
        parser.add_argument('-d', '--dropoff', required=True,
                            type=str,
                            help='Dropoff waypoint name')
        parser.add_argument(
            '--duration_secs',
            help='Estimated action duration in seconds, default: 10',
            type=int,
            default=10,
        )
        parser.add_argument(
            '--use_tool_sink',
            action='store_true',
            help='Use tool sink during perform action, default: false',
        )

    def build_request(self, args):
        """Build a cart delivery request."""
        def perform_action(action_category):
            return {
                'category': 'perform_action',
                'description': {
                    'unix_millis_action_duration_estimate': (
                        args.duration_secs * 1000
                    ),
                    # for internal FleetUpdateHandle to check if action
                    # is performable by this fleet
                    'category': action_category,
                    'description': {
                        # for fleet manager to start action process
                        'deliver_cart_task_name': action_category
                    },
                    'use_tool_sink': args.use_tool_sink,
                },
            }

        activities = [
            {'category': 'go_to_place', 'description': args.pickup},
            perform_action('delivery_pickup'),
            {'category': 'go_to_place', 'description': args.dropoff},
            perform_action('delivery_dropoff'),
        ]
        return {
            'category': 'compose',
            'description': task_api.compose_sequence(
                'deliver_cart', activities
            ),
        }


def main(argv=sys.argv):
    """Dispatch a cart delivery."""
    TaskRequester.run(argv)


if __name__ == '__main__':
    main(sys.argv)
