#!/usr/bin/env python3

# Copyright 2024 Open Source Robotics Foundation, Inc.
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
"""Dispatch any json task description."""

import sys

from rmf_demos_tasks import task_api


class TaskRequester(task_api.TaskRequester):
    """Task requester."""

    request_prefix = 'task'
    supports_estimate = True

    def add_arguments(self, parser):
        """Add json task arguments."""
        parser.add_argument(
            '-f', '--file',
            required=True,
            type=str,
            help=(
                'File containing a json object containing at least '
                '{{ "category": -, "description": - }}. The contents of this '
                'file will be inserted into the request message.'
            ),
        )

    def build_request(self, args):
        """Insert the user-provided contents into the request."""
        return task_api.read_json_object(args.file)


def main(argv=sys.argv):
    """Dispatch any json-defined task category and description."""
    TaskRequester.run(argv)


if __name__ == '__main__':
    main(sys.argv)
