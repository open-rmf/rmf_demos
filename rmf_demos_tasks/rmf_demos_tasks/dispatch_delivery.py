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
"""Dispatch a delivery."""

import sys

from rmf_demos_tasks import task_api


def payload_from_arg(sku_qty_arg, kind):
    """Turn an optional 'sku,quantity' argument into a delivery payload."""
    if sku_qty_arg is None:
        return []
    sku_qty = sku_qty_arg.split(',')
    assert len(sku_qty) == 2, f'please specify sku and qty for {kind} payload'
    return [{'sku': sku_qty[0], 'quantity': int(sku_qty[1])}]


class TaskRequester(task_api.TaskRequester):
    """Task requester."""

    request_prefix = 'delivery'
    supports_estimate = True

    def add_arguments(self, parser):
        """Add delivery arguments."""
        parser.add_argument(
            '-p',
            '--pickups',
            required=True,
            type=str,
            nargs='+',
            help='Pickup names',
        )
        parser.add_argument(
            '-d',
            '--dropoffs',
            required=True,
            type=str,
            nargs='+',
            help='Dropoff names',
        )
        parser.add_argument(
            '-ph',
            '--pickup_handlers',
            required=True,
            type=str,
            nargs='+',
            help='Pickup handler names',
        )
        parser.add_argument(
            '-dh',
            '--dropoff_handlers',
            required=True,
            type=str,
            nargs='+',
            help='Dropoffs handler names',
        )
        parser.add_argument(
            '-pp',
            '--pickup_payloads',
            type=str,
            nargs='+',
            default=[],
            help='Pickup payload [sku,quantity sku2,qty...]',
        )
        parser.add_argument(
            '-dp',
            '--dropoff_payloads',
            type=str,
            nargs='+',
            default=[],
            help='Dropoff payload [sku,quantity sku2,qty...]',
        )

    def build_request(self, args):
        """Build a delivery or multi_delivery request."""
        # check user delivery arg inputs
        if len(args.pickups) != len(args.pickup_handlers):
            self.get_logger().error(
                'Invalid pickups, [-p] should have the same length as [-ph]'
            )
            self.parser.print_help()
            sys.exit(1)
        if len(args.dropoffs) != len(args.dropoff_handlers):
            self.get_logger().error(
                'Invalid dropoffs, [-d] should have the same length as [-dh]'
            )
            self.parser.print_help()
            sys.exit(1)

        def place(kind, places, handlers, payloads, index):
            sku_qty = payloads[index] if index < len(payloads) else None
            return {
                'place': places[index],
                'handler': handlers[index],
                'payload': payload_from_arg(sku_qty, kind),
            }

        def pickup(index):
            return place('pickup', args.pickups, args.pickup_handlers,
                         args.pickup_payloads, index)

        def dropoff(index):
            return place('dropoff', args.dropoffs, args.dropoff_handlers,
                         args.dropoff_payloads, index)

        # Use standard delivery task type
        if len(args.pickups) == 1 and len(args.dropoffs) == 1:
            return {
                'category': 'delivery',
                'description': {'pickup': pickup(0), 'dropoff': dropoff(0)},
            }

        # Define multi_delivery with request category compose
        activities = []
        for i in range(len(args.pickups)):
            activities.append({'category': 'pickup', 'description': pickup(i)})
        for i in range(len(args.dropoffs)):
            activities.append(
                {'category': 'dropoff', 'description': dropoff(i)}
            )
        return {
            'category': 'compose',
            'description': task_api.compose_sequence(
                'multi_delivery', activities
            ),
        }


def main(argv=sys.argv):
    """Dispatch a delivery."""
    TaskRequester.run(argv)


if __name__ == '__main__':
    main(sys.argv)
