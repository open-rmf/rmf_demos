#!/usr/bin/env python3

# Copyright 2025 Open Source Robotics Foundation, Inc.
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
"""Dynamic Event."""

import json
import sys

from rmf_demos_tasks import task_api


class TaskRequester(task_api.TaskRequester):
    """Task requester."""

    request_prefix = 'direct'

    def add_arguments(self, parser):
        """Add dynamic event arguments."""
        parser.add_argument(
            '-p',
            '--place',
            type=str,
            nargs='+',
            help=(
                'Place(s) for the fleet adapter to estimate the robot will '
                'go. If more than one is listed, the fleet adapter will '
                'choose the closest reachable place. This cannot be used with '
                '--file.'
            ),
        )
        parser.add_argument(
            '-f',
            '--file',
            type=str,
            help=(
                'File containing a json object containing at least '
                '{{ "category": -, "description": - }}. The contents of this '
                'file will be used as the estimate of the dynamic event. This '
                'cannot be used with --place.'
            ),
        )
        parser.add_argument(
            '-r',
            '--required',
            type=str,
            nargs='+',
            help=(
                'Add an action capability requirement for the selected fleet. '
                'This takes the form of a file containing '
                '{{ "category": -, "description": - }}, similar to --file. '
                'The fleet must be able to execute the described event. More '
                'than one file can be specified and each one will be required.'
            ),
        )
        parser.add_argument(
            '--parameters',
            type=str,
            help=(
                'File containing a json object for the parameters of the '
                'dynamic event. These parameters will be published with the '
                'dynamic event information when the event begins.'
            ),
        )
        parser.add_argument(
            '--detail',
            type=str,
            help='Human-friendly detailed description of the dynamic event.',
        )

    def build_request(self, args):
        """Build a dynamic event request."""
        if args.file and args.place:
            raise RuntimeError(
                'You cannot use both --file and --place at the same time.'
            )

        description = {
            'required': [
                task_api.read_json_object(f) for f in args.required or []
            ],
        }

        if args.place:
            description['category'] = 'go_to_place'
            description['estimate'] = {
                'category': 'go_to_place',
                'description': {'one_of': args.place},
            }

        if args.file:
            estimate = task_api.read_json_object(args.file)
            description['category'] = estimate['category']
            description['estimate'] = estimate

        if args.parameters:
            with open(args.parameters) as f:
                description['parameters'] = json.load(f)

        if args.detail:
            description['detail'] = args.detail

        dynamic_event_activity = {
            'category': 'dynamic_event',
            'description': description,
        }
        return {
            'category': 'compose',
            'description': {
                'category': 'dynamic_event',
                'phases': [{'activity': dynamic_event_activity}],
            },
        }


def main(argv=sys.argv):
    """Dispatch a dynamic event."""
    TaskRequester.run(argv)


if __name__ == '__main__':
    main(sys.argv)
