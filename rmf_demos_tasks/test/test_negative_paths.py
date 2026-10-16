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
"""
Negative-path tests for the task requester feature surface.

Covers failure modes that the happy-path test in test_future.py does not
touch:
  * dispatch_json: file-content validation and the --estimate guard
  * dispatch_go_to_place: the --estimate guard and --start-at-departure
  * RobotStateObserver (get_robot_location): fleet/floor/tolerance
    filtering and --block-until-reaches
  * TaskObserver (wait_for_task_complete): non-matching task states

The payload builders (``build_payload``) are module-level functions and the
observer callbacks are exercised as unbound methods with lightweight
stubs, so no publisher or live RMF stack is needed; a running rclpy
context (as in test_future.py) is still required only because the modules
import rclpy at top level.
"""

from types import SimpleNamespace

import pytest

from rmf_demos_tasks.dispatch_go_to_place import (
    build_payload as goto_build_payload,
    get_args_parser as goto_get_args_parser,
)
from rmf_demos_tasks.dispatch_json import (
    build_payload as json_build_payload,
    get_args_parser as json_get_args_parser,
)
from rmf_demos_tasks.get_robot_location import RobotStateObserver
from rmf_demos_tasks.wait_for_task_complete import TaskObserver


class _FutureStub:
    """Minimal stand-in for rclpy.Future / asyncio.Future."""

    def __init__(self):
        self._result = None
        self._done = False

    def done(self):
        """Return whether a result has been set."""
        return self._done

    def set_result(self, result):
        """Record a result."""
        self._result = result
        self._done = True

    def result(self):
        """Return the recorded result."""
        return self._result


def _observe_unbound(method, self_stub, *args):
    """Call a Node callback as an unbound method against a plain stub."""
    method(self_stub, *args)
    return self_stub


# ---------------------------------------------------------------------------
# dispatch_json: build_payload negative paths
# ---------------------------------------------------------------------------

def _json_args(*extra):
    args_list = ['-f', 'dummy.json'] + list(extra)
    return json_get_args_parser().parse_args(args_list)


def test_json_missing_category_raises():
    """The file object must contain a category field."""
    with pytest.raises(RuntimeError, match='"category"'):
        json_build_payload(_json_args(), 1000, {'description': {}})


def test_json_missing_description_raises():
    """The file object must contain a description field."""
    with pytest.raises(RuntimeError, match='"description"'):
        json_build_payload(_json_args(), 1000, {'category': 'patrol'})


def test_json_estimate_without_fleet_or_robot_raises():
    """--estimate requires both fleet and robot names."""
    with pytest.raises(RuntimeError, match='--estimate'):
        json_build_payload(_json_args('-e'), 1000,
                           {'category': 'c', 'description': {}})


def test_json_estimate_with_fleet_but_no_robot_raises():
    """The guard requires both fleet AND robot, not just one."""
    with pytest.raises(RuntimeError, match='--estimate'):
        json_build_payload(_json_args('-F', 'fleet1', '-e'), 1000,
                           {'category': 'c', 'description': {}})


def test_json_estimate_with_both_uses_estimate_type():
    """Positive control: the guard passes and the estimate type is used."""
    payload = json_build_payload(
        _json_args('-F', 'fleet1', '-R', 'robot1', '-e'), 1000,
        {'category': 'c', 'description': {}})
    assert payload['type'] == 'estimate_robot_task_request'
    assert payload['fleet'] == 'fleet1'
    assert payload['robot'] == 'robot1'


def test_json_dispatch_type_with_fleet_only():
    """Positive control: fleet without robot falls back to dispatch type."""
    payload = json_build_payload(
        _json_args('-F', 'fleet1'), 1234,
        {'category': 'patrol', 'description': {'places': ['a']}})
    assert payload['type'] == 'dispatch_task_request'
    assert payload['request']['category'] == 'patrol'
    assert payload['request']['unix_millis_earliest_start_time'] == 1234


def test_json_file_contents_still_win_over_defaults():
    """A file-specified category/description must survive request.update()."""
    payload = json_build_payload(
        _json_args(), 1000,
        {'category': 'patrol', 'description': {'rounds': 2}})
    assert payload['request']['category'] == 'patrol'
    assert payload['request']['description'] == {'rounds': 2}


def test_json_file_contents_merged_into_request():
    """File fields are merged into the request without losing defaults."""
    payload = json_build_payload(
        _json_args(), 1000,
        {'category': 'patrol', 'description': {'rounds': 2}})
    request = payload['request']
    assert request['category'] == 'patrol'
    assert request['description'] == {'rounds': 2}
    assert 'unix_millis_earliest_start_time' in request


# ---------------------------------------------------------------------------
# dispatch_go_to_place: build_payload negative paths
# ---------------------------------------------------------------------------

def test_goto_estimate_without_robot_raises():
    """--estimate requires both fleet and robot names."""
    args = goto_get_args_parser().parse_args(
        ['-p', 'place1', '-F', 'fleet1', '-e'])
    with pytest.raises(RuntimeError, match='--estimate'):
        goto_build_payload(args, 1000)


def test_goto_estimate_without_fleet_raises():
    """--estimate requires both fleet and robot names."""
    args = goto_get_args_parser().parse_args(
        ['-p', 'place1', '-R', 'robot1', '-e'])
    with pytest.raises(RuntimeError, match='--estimate'):
        goto_build_payload(args, 1000)


def test_goto_estimate_with_both_uses_estimate_type():
    """Positive control: estimate type is selected when both are given."""
    args = goto_get_args_parser().parse_args(
        ['-p', 'place1', '-F', 'fleet1', '-R', 'robot1', '-e'])
    payload = goto_build_payload(args, 1000)
    assert payload['type'] == 'estimate_robot_task_request'


def test_goto_start_at_departure_default_absent():
    """By default the description must NOT contain start_at_departure."""
    args = goto_get_args_parser().parse_args(['-p', 'place1'])
    payload = goto_build_payload(args, 1000)
    description = (
        payload['request']['description']['phases'][0]['activity']
        ['description'])
    assert 'start_at_departure' not in description


def test_goto_start_at_departure_flag_sets_true():
    """--start-at-departure must set start_at_departure flag to true."""
    args = goto_get_args_parser().parse_args(
        ['-p', 'place1', '--start-at-departure'])
    payload = goto_build_payload(args, 1000)
    description = (
        payload['request']['description']['phases'][0]['activity']
        ['description'])
    assert description['start_at_departure'] is True
    # existing fields must survive alongside it
    assert description['one_of'] == [{'waypoint': 'place1'}]


# ---------------------------------------------------------------------------
# RobotStateObserver (get_robot_location.py) — unbound callback testing
# ---------------------------------------------------------------------------

def _obs_stub(fleet='fleet1', robot='robot1', tolerance=0.5,
              block_until_reaches=None):
    parser = SimpleNamespace(
        fleet=fleet, robot=robot, tolerance=tolerance,
        block_until_reaches=block_until_reaches)
    # tolerance_squared is computed in RobotStateObserver.__init__ and read
    # by state_watcher; replicate that derivation on the stub.
    stub = SimpleNamespace(
        parser=parser, response=_FutureStub(), nav_graph=None,
        tolerance_squared=tolerance ** 2)
    return stub


def _graph(fleet='fleet1', vertices=(), name='fleet1'):
    return SimpleNamespace(name=name, vertices=list(vertices))


def _graph_node(name, x, y, floor='L1'):
    return SimpleNamespace(
        name=name, x=x, y=y,
        params=[SimpleNamespace(name='map_name', value_string=floor)])


def _fleet_state(name='fleet1', robots=()):
    return SimpleNamespace(name=name, robots=list(robots))


def _robot(name='robot1', x=0.0, y=0.0, level='L1'):
    return SimpleNamespace(
        name=name, location=SimpleNamespace(x=x, y=y, level_name=level))


def test_robs_wrong_fleet_ignored():
    """A fleet state for another fleet must not resolve the response."""
    stub = _obs_stub()
    stub.nav_graph = _graph(vertices=[_graph_node('wp', 0.0, 0.0)])
    _observe_unbound(
        RobotStateObserver.state_watcher, stub,
        _fleet_state(name='other_fleet', robots=[_robot()]))
    assert not stub.response.done()


def test_robs_wrong_robot_ignored():
    """A state for another robot must not resolve the response."""
    stub = _obs_stub()
    stub.nav_graph = _graph(vertices=[_graph_node('wp', 0.0, 0.0)])
    _observe_unbound(
        RobotStateObserver.state_watcher, stub,
        _fleet_state(robots=[_robot(name='other_robot')]))
    assert not stub.response.done()


def test_robs_no_nav_graph_no_result():
    """Without a nav graph yet, no result may be set."""
    stub = _obs_stub()
    _observe_unbound(
        RobotStateObserver.state_watcher, stub,
        _fleet_state(robots=[_robot()]))
    assert not stub.response.done()


def test_robs_wrong_floor_skipped():
    """A closer vertex on a different floor must not be selected."""
    stub = _obs_stub()
    stub.nav_graph = _graph(vertices=[
        _graph_node('wrong_floor_wp', -0.1, 0.0, floor='L2'),
        _graph_node('right_floor_wp', 0.4, 0.0, floor='L1'),
    ])
    _observe_unbound(
        RobotStateObserver.state_watcher, stub,
        _fleet_state(robots=[_robot()]))
    assert stub.response.done()
    assert stub.response.result() == 'right_floor_wp'


def test_robs_outside_tolerance_no_result():
    """A vertex outside the tolerance radius must not resolve the response."""
    stub = _obs_stub(tolerance=0.5)
    stub.nav_graph = _graph(vertices=[_graph_node('far_wp', 0.6, 0.0)])
    _observe_unbound(
        RobotStateObserver.state_watcher, stub,
        _fleet_state(robots=[_robot()]))
    assert not stub.response.done()


def test_robs_inclusive_tolerance_boundary_selected():
    """A vertex exactly at the tolerance distance must be selected (<=)."""
    stub = _obs_stub(tolerance=0.5)
    stub.nav_graph = _graph(vertices=[_graph_node('edge_wp', 0.5, 0.0)])
    _observe_unbound(
        RobotStateObserver.state_watcher, stub,
        _fleet_state(robots=[_robot()]))
    assert stub.response.done()
    assert stub.response.result() == 'edge_wp'


def test_robs_closest_in_range_wins():
    """Among in-tolerance vertices the strictly closer one must win."""
    stub = _obs_stub(tolerance=1.0)
    stub.nav_graph = _graph(vertices=[
        _graph_node('far', 0.9, 0.0),
        _graph_node('near', 0.1, 0.0),
    ])
    _observe_unbound(
        RobotStateObserver.state_watcher, stub,
        _fleet_state(robots=[_robot()]))
    assert stub.response.result() == 'near'


def test_robs_block_until_mismatch_ignored():
    """Reaching a different waypoint must not satisfy a blocked future."""
    stub = _obs_stub(block_until_reaches='target_wp')
    stub.nav_graph = _graph(vertices=[_graph_node('other_wp', 0.1, 0.0)])
    _observe_unbound(
        RobotStateObserver.state_watcher, stub,
        _fleet_state(robots=[_robot()]))
    assert not stub.response.done()


def test_robs_block_until_match_resolves():
    """Positive control: reaching the blocked waypoint resolves the future."""
    stub = _obs_stub(block_until_reaches='target_wp')
    stub.nav_graph = _graph(vertices=[_graph_node('target_wp', 0.1, 0.0)])
    _observe_unbound(
        RobotStateObserver.state_watcher, stub,
        _fleet_state(robots=[_robot()]))
    assert stub.response.done()
    assert stub.response.result() == 'target_wp'


def test_robs_nav_graph_unknown_waypoint_sets_error_once():
    """
    An unknown --block-until-reaches waypoint yields the error string.

    A repeated nav graph update must not overwrite the set error.
    """
    stub = _obs_stub(block_until_reaches='no_such_wp')
    _observe_unbound(
        RobotStateObserver.nav_graph_watcher, stub, _graph())
    assert stub.response.done()
    assert 'no_such_wp' in stub.response.result()
    first = stub.response.result()
    _observe_unbound(
        RobotStateObserver.nav_graph_watcher, stub, _graph())
    assert stub.response.result() is first


# ---------------------------------------------------------------------------
# TaskObserver (wait_for_task_complete.py) — unbound callback testing
# ---------------------------------------------------------------------------

def _task_stub(fleet='fleet1', robot='robot1'):
    return SimpleNamespace(
        parser=SimpleNamespace(fleet=fleet, robot=robot),
        response=_FutureStub())


def test_task_wrong_fleet_ignored():
    """A fleet state for another fleet must not resolve the response."""
    stub = _task_stub()
    _observe_unbound(
        TaskObserver.state_watcher, stub,
        _fleet_state(name='other_fleet', robots=[
            SimpleNamespace(name='robot1', task_id='')]))
    assert not stub.response.done()


def test_task_still_running_ignored():
    """A non-empty task_id means the task is not complete: no result."""
    stub = _task_stub()
    _observe_unbound(
        TaskObserver.state_watcher, stub,
        _fleet_state(robots=[SimpleNamespace(name='robot1', task_id='t1')]))
    assert not stub.response.done()


def test_task_complete_resolves():
    """Positive control: task_id == '' resolves with the robot state."""
    stub = _task_stub()
    robot = SimpleNamespace(name='robot1', task_id='')
    _observe_unbound(
        TaskObserver.state_watcher, stub,
        _fleet_state(robots=[robot]))
    assert stub.response.done()
    assert stub.response.result() is robot


if __name__ == '__main__':
    raise SystemExit(pytest.main([__file__, '-v']))
