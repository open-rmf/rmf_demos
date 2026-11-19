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
"""Cancel a task."""

import argparse
import sys
import uuid

from rmf_demos_tasks import task_api


class TaskRequester(task_api.ApiRequester):
    """Task requester."""

    def __init__(self, argv=sys.argv):
        """Initialize a task requester."""
        parser = argparse.ArgumentParser()
        parser.add_argument(
            '-id',
            '--task_id',
            required=True,
            default='',
            type=str,
            help='Cancel Task ID',
        )
        task_api.add_timeout_argument(parser)
        self.args = parser.parse_args(argv[1:])

        super().__init__()
        self.send(
            f'cancel_task_{uuid.uuid4()}',
            {'type': 'cancel_task_request', 'task_id': self.args.task_id},
        )


def main(argv=sys.argv):
    """Cancel a task."""
    TaskRequester.run(argv)


if __name__ == '__main__':
    main(sys.argv)
