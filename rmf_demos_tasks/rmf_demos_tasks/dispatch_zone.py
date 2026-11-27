#!/usr/bin/env python3

# Copyright 2026 Open Source Robotics Foundation, Inc.
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
"""Dispatch a zone task."""

import argparse
import sys

from rmf_demos_tasks import task_api


def parse_hint(text):
    """Turn one -H argument into the hint object the zone schema takes."""
    parts = text.split(':')
    kind = parts[0]
    name = ''
    if len(parts) > 1:
        name = parts[1]
    hint = {}
    if kind == 'wp':
        hint['waypoint'] = name
        rest = parts[2:]
    elif kind == 'group':
        hint['group'] = name
        rest = parts[2:]
    elif kind == 'any':
        rest = parts[1:]
    else:
        raise argparse.ArgumentTypeError(f'unknown hint type [{kind}]')
    if kind != 'any' and not name:
        raise argparse.ArgumentTypeError(f'{kind} needs a name')
    if len(rest) > 1:
        raise argparse.ArgumentTypeError(
            'orientations go in one comma separated list')
    if rest and rest[0]:
        hint['orientations'] = []
        for angle in rest[0].split(','):
            hint['orientations'].append(float(angle))
    return hint


class TaskRequester(task_api.TaskRequester):
    """Dispatch zone task requester."""

    request_prefix = 'zone'

    def add_arguments(self, parser):
        """Add zone arguments."""
        parser.add_argument(
            '-z',
            '--zone',
            required=True,
            type=str,
            help='Name of the target zone',
        )
        parser.add_argument(
            '-H',
            '--hint',
            action='append',
            type=parse_hint,
            default=None,
            metavar='TYPE[:NAME][:ANGLE,...]',
            help='Add a preference, most preferred first. TYPE is wp, '
                 'group or any, and any takes no name. Orientations are '
                 'radians. Examples: wp:wait_1:1.57,4.71  group:C:0.7  '
                 'any:1.57',
        )

    def build_request(self, args):
        """Build a zone request."""
        description = {'zone_name': args.zone}
        if args.hint:
            description['hints'] = args.hint
        return {'category': 'zone', 'description': description}


def main(argv=sys.argv):
    """Dispatch a zone task."""
    TaskRequester.run(argv)


if __name__ == '__main__':
    main(sys.argv)
