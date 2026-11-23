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
"""Go to place."""

import math
import sys

from rmf_demos_tasks import task_api


class TaskRequester(task_api.TaskRequester):
    """Task requester."""

    request_prefix = 'direct'
    supports_estimate = True

    def add_arguments(self, parser):
        """Add go to place arguments."""
        parser.add_argument(
            '-p',
            '--place',
            required=True,
            type=str,
            help='Place to go to',
            nargs='+',
        )
        parser.add_argument(
            '-o',
            '--orientation',
            required=False,
            type=float,
            help='Orientation to face in degrees (optional)',
        )
        parser.add_argument(
            '-m',
            '--prefer-same-map',
            required=False,
            action='store_true',
            help=(
                'When choosing between multiple destination options, prefer '
                'an option on the same map as the starting location.'
            ),
        )
        parser.add_argument(
            '--start-at-departure',
            action='store_true',
            help=(
                'Request the robot to not start going to its destination '
                'until the task start time has been reached, rather than '
                'trying to arrive at the destination at the task start time.'
            )
        )

    def build_request(self, args):
        """Build a go to place request."""
        one_of = []
        for place in args.place:
            place_json = {'waypoint': place}
            if args.orientation is not None:
                place_json['orientation'] = args.orientation * math.pi / 180.0
            one_of.append(place_json)

        go_to_description = {'one_of': one_of}
        if args.prefer_same_map:
            go_to_description['constraints'] = [
                {'category': 'prefer_same_map'}
            ]
        if args.start_at_departure:
            go_to_description['start_at_departure'] = True

        go_to_activity = {
            'category': 'go_to_place',
            'description': go_to_description,
        }
        return {
            'category': 'compose',
            'description': {
                'category': 'go_to_place',
                'phases': [{'activity': go_to_activity}],
            },
        }


def main(argv=sys.argv):
    """Go to place."""
    TaskRequester.run(argv)


if __name__ == '__main__':
    main(sys.argv)
