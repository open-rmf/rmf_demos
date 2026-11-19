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
"""Dispatch a patrol task."""

import sys

from rmf_demos_tasks import task_api


class TaskRequester(task_api.TaskRequester):
    """Dispatch patrol task requester."""

    request_prefix = 'patrol'

    def add_arguments(self, parser):
        """Add patrol arguments."""
        parser.add_argument(
            '-p',
            '--places',
            required=True,
            nargs='+',
            type=str,
            help='Places to patrol through',
        )
        parser.add_argument(
            '-n',
            '--rounds',
            help='Number of loops to perform',
            type=int,
            default=1,
        )

    def build_request(self, args):
        """Build a patrol request."""
        return {
            'category': 'patrol',
            'description': {'places': args.places, 'rounds': args.rounds},
        }


def main(argv=sys.argv):
    """Dispatch a patrol task."""
    TaskRequester.run(argv)


if __name__ == '__main__':
    main(sys.argv)
