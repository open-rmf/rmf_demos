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
"""Check that task requesters can be spun with rclpy."""

import rclpy
from rclpy.task import Future

from rmf_demos_tasks.dispatch_patrol import TaskRequester


def test_dispatch_patrol_response_is_rclpy_future():
    """Spin until the dispatch_patrol response future times out."""
    rclpy.init()
    requester = None
    try:
        requester = TaskRequester(['dispatch_patrol', '-p', 'test_place'])
        assert isinstance(requester.response, Future)

        rclpy.spin_until_future_complete(
            requester, requester.response, timeout_sec=0.1
        )
        assert not requester.response.done()
    finally:
        if requester is not None:
            requester.destroy_node()
        rclpy.shutdown()
