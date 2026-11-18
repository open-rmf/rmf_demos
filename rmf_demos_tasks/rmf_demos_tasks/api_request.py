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
"""Send a generic API request."""

import argparse
import json
import sys
import uuid

from rmf_demos_tasks import task_api


class ApiRequester(task_api.ApiRequester):
    """API requester."""

    def __init__(self, argv=sys.argv):
        """Initialize an API requester."""
        parser = argparse.ArgumentParser()
        parser.add_argument(
            '-f', '--file',
            required=True,
            type=str,
            help='File containing a task description formatted in json',
        )
        parser.add_argument(
            '--id',
            type=str,
            help='Specify an ID for this request',
        )
        task_api.add_timeout_argument(parser)
        self.args = parser.parse_args(argv[1:])

        with open(self.args.file) as f:
            payload = json.load(f)

        super().__init__()
        self.send(self.args.id or f'request_{uuid.uuid4()}', payload)


def main(argv=sys.argv):
    """Send a generic API request."""
    ApiRequester.run(argv)


if __name__ == '__main__':
    main(sys.argv)
