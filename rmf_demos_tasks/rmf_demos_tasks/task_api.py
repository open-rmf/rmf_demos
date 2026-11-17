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
"""Shared helpers for the task API command line tools."""

import argparse
import json
import sys
import uuid

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSDurabilityPolicy as Durability
from rclpy.qos import QoSHistoryPolicy as History
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy as Reliability

from rmf_task_msgs.msg import ApiRequest
from rmf_task_msgs.msg import ApiResponse


def add_timeout_argument(parser):
    """Add the --timeout argument used by every requester."""
    parser.add_argument(
        '--timeout',
        type=float,
        default=5.0,
        help='Timeout in seconds to wait for a response, default: 5.0',
    )


def compose_sequence(category, activities):
    """Make a compose task description that runs activities in sequence."""
    return {
        'category': category,
        'phases': [
            {
                'activity': {
                    'category': 'sequence',
                    'description': {'activities': activities},
                }
            }
        ],
    }


def read_json_object(path, what='input json file'):
    """Load a json object that must have a category and a description."""
    with open(path) as f:
        contents = json.load(f)
    if 'category' not in contents or 'description' not in contents:
        raise RuntimeError(
            f'The {what} {path} must have an object that contains both a '
            '"category" and a "description" field.'
        )
    return contents


class ApiRequester(Node):
    """Send one request on the task API and wait for its response."""

    def __init__(self, node_name='task_requester'):
        """Create the request publisher and response subscription."""
        super().__init__(node_name)
        self.response = rclpy.Future()
        self._request_id = None

        transient_qos = QoSProfile(
            history=History.KEEP_LAST,
            depth=1,
            reliability=Reliability.RELIABLE,
            durability=Durability.TRANSIENT_LOCAL,
        )
        self.pub = self.create_publisher(
            ApiRequest, 'task_api_requests', transient_qos
        )

        # Responders publish responses as transient local, so a response sent
        # before discovery has matched this subscription is still delivered.
        # Old responses are ignored because they have a different request id.
        response_qos = QoSProfile(
            history=History.KEEP_LAST,
            depth=10,
            reliability=Reliability.RELIABLE,
            durability=Durability.TRANSIENT_LOCAL,
        )
        self.sub = self.create_subscription(
            ApiResponse,
            'task_api_responses',
            self._receive_response,
            response_qos,
        )

    def _receive_response(self, response_msg: ApiResponse):
        if response_msg.request_id == self._request_id:
            self.response.set_result(json.loads(response_msg.json_msg))

    def send(self, request_id, payload):
        """Publish a json payload with the given request id."""
        self._request_id = request_id
        msg = ApiRequest()
        msg.request_id = request_id
        msg.json_msg = json.dumps(payload)
        print(f'Json msg payload: \n{json.dumps(payload, indent=2)}')
        self.pub.publish(msg)

    @classmethod
    def run(cls, argv=sys.argv):
        """
        Run a requester as a command line tool.

        This must be called on a subclass whose constructor takes the command
        line arguments (with ROS arguments removed), sets self.args with a
        timeout field, and sends its request.
        """
        if cls is ApiRequester:
            raise TypeError(
                'ApiRequester.run() must be called on a subclass whose '
                'constructor takes argv'
            )

        rclpy.init(args=argv)
        requester = None
        got_response = False
        try:
            requester = cls(rclpy.utilities.remove_ros_args(argv))
            rclpy.spin_until_future_complete(
                requester,
                requester.response,
                timeout_sec=requester.args.timeout,
            )
            got_response = requester.response.done()
            if got_response:
                print(f'Got response: \n{requester.response.result()}')
            else:
                print('Did not get a response')
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
        finally:
            if requester is not None:
                requester.destroy_node()
            # Also destroys a node whose constructor raised part way through.
            rclpy.try_shutdown()

        if not got_response:
            sys.exit(1)


class TaskRequester(ApiRequester):
    """
    Base class for the dispatch_* tools.

    Subclasses set request_prefix, add their own arguments in add_arguments
    and return the category and description of the task in build_request.
    """

    request_prefix = 'task'
    supports_estimate = False

    def __init__(self, argv=sys.argv):
        """Parse arguments, then build and send the task request."""
        self.parser = argparse.ArgumentParser()
        self.add_arguments(self.parser)
        self._add_common_arguments(self.parser)
        args = self.parser.parse_args(argv[1:])

        super().__init__()
        self.args = args

        if self.args.use_sim_time:
            self.get_logger().info('Using Sim Time')
            param = Parameter('use_sim_time', Parameter.Type.BOOL, True)
            self.set_parameters([param])

        payload = self._make_payload()
        request = self._make_request()
        request.update(self.build_request(self.args))
        payload['request'] = request

        request_id = f'{self.request_prefix}_{uuid.uuid4()}'
        self.send(request_id, payload)

    def add_arguments(self, parser):
        """Add the arguments that are specific to this task."""

    def build_request(self, args):
        """Return the task-specific fields of the request."""
        raise NotImplementedError

    def _add_common_arguments(self, parser):
        parser.add_argument(
            '-F',
            '--fleet',
            type=str,
            help='Fleet name, should define together with robot',
        )
        parser.add_argument(
            '-R',
            '--robot',
            type=str,
            help='Robot name, should define together with fleet',
        )
        parser.add_argument(
            '-st',
            '--start_time',
            help='Start time from now in secs, default: 0',
            type=int,
            default=0,
        )
        parser.add_argument(
            '-pt',
            '--priority',
            help='Priority value for this request (0 or 1)',
            type=int,
            default=None,
        )
        parser.add_argument(
            '--use_sim_time',
            action='store_true',
            help='Use sim time, default: false',
        )
        parser.add_argument(
            '--requester',
            help='Entity that is requesting this task',
            type=str,
            default='rmf_demos_tasks',
        )
        if self.supports_estimate:
            parser.add_argument(
                '-e',
                '--estimate',
                action='store_true',
                help=(
                    'Request an estimate instead of dispatching a task. '
                    'You must specify both the fleet and robot names for '
                    'this setting.'
                ),
            )
        add_timeout_argument(parser)

    def _make_payload(self):
        estimate = getattr(self.args, 'estimate', False)
        payload = {}
        if self.args.fleet and self.args.robot:
            if estimate:
                payload['type'] = 'estimate_robot_task_request'
            else:
                payload['type'] = 'robot_task_request'
            payload['robot'] = self.args.robot
            payload['fleet'] = self.args.fleet
        else:
            if estimate:
                raise RuntimeError(
                    'Cannot use --estimate without specifying names for both '
                    'fleet and robot'
                )
            payload['type'] = 'dispatch_task_request'
        self.get_logger().info(f"Using '{payload['type']}'")
        return payload

    def _make_request(self):
        now = self.get_clock().now().to_msg()
        now.sec = now.sec + self.args.start_time
        start_time = now.sec * 1000 + round(now.nanosec / 10**6)
        request = {
            'unix_millis_request_time': start_time,
            'unix_millis_earliest_start_time': start_time,
            'requester': self.args.requester,
        }
        if self.args.priority is not None:
            request['priority'] = {
                'type': 'binary',
                'value': 1 if self.args.priority > 0 else 0,
            }
        if self.args.fleet:
            request['fleet_name'] = self.args.fleet
        return request
