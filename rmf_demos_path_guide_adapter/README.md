# rmf_demos_path_guide_adapter

A Path Guide fleet adapter for the RMF Demos robots.

This package is modelled on `rmf_demos_fleet_adapter` and does not modify it.
Run either one; they are two independent demos of the same simulated robots.

## What is different

`rmf_demos_fleet_adapter` uses `EasyFullControl`, which hands the fleet manager
**one destination at a time** and waits for `execution.finished()` before
revealing the next one. Two consequences:

- the robot never sees its route, so a fleet manager cannot smooth, spline or
  velocity-profile across it;
- every vertex costs a robot → manager → adapter → manager → robot round trip
  before motion continues.

This package uses `PathGuide`, which hands over the **entire remaining path** in
one call. The **fleet manager** then measures the robot's position against each
waypoint's `arrival_radius` and reports the furthest waypoint reached as
`reached_waypoint_index` on `/status`; the adapter acknowledges up to that index
and measures nothing itself.

```
EasyFullControl:  navigate(destination, execution)   ... once per vertex
PathGuide:        follow_path(path)                  ... once per plan
                    -> path.waypoints[i].execution.finished()   per vertex
```

## Acknowledgement ordering

`PathGuide` accepts `finished()` **only** for the waypoint it is currently
tracking. An out-of-order call is rejected with a warning and the path does not
advance.

If the robot passes a vertex without the arrival check firing, the missed
waypoints must be acknowledged **in sequence** before the one it actually
reached. The fleet manager reports a *level* — the furthest waypoint reached —
while PathGuide's contract is expressed in *edges*, so
`RobotAdapter.acknowledge_through` walks from `current_index` up to that index
one `finished()` at a time.

`FleetManager.advance_reached_index` is what picks that index, and it finds the
**furthest** waypoint reached rather than stopping at the first one not
reached — the naive alternative deadlocks in exactly this case, since a
waypoint the robot never came close to is only ever credited via a later one.

## Reported index must never go backwards

Whichever waypoint's `identifier` this adapter passes to
`update_handle.update()` is what RMF records as the index the robot is
approaching, and **it may never decrease for a given path**
([rmf_ros2#530](https://github.com/open-rmf/rmf_ros2/issues/530#issuecomment-5099640879)).
RMF converts it into an itinerary delay without clamping, so a decrease makes
this robot look like it is still occupying space it has left, which can deadlock
others.

`PathGuide` enforces this on the C++ side. On violation it logs an error, stops
the robot and requests a replan — deliberately not a silent clamp, since a real
physical backtrack is the case that must not be hidden.

This adapter is safe by construction: it reports
`current_waypoint().execution.identifier`, and `current_index` only increases.
The trap is reporting the **nearest** waypoint each tick — the obvious
implementation, and one that backtracks as soon as a robot reverses or loops
around an obstacle.

## Running it

This package ships one launch file, `launch/path_guide_adapter.launch.xml`,
which brings up the fleet manager and the adapter for a single fleet:

```bash
ros2 launch rmf_demos_path_guide_adapter path_guide_adapter.launch.xml \
    config_file:=<your fleet config> \
    nav_graph_file:=<your nav graph>
```

`config.yaml` here is a template, shipped for reference; nothing loads it by
default. It mirrors the stock `tinyRobot` fleet so that comparing it against
`rmf_demos_fleet_adapter`'s config shows only the Path Guide differences.

Wiring this into a simulated world — a fleet config, a nav graph, and the
world's own launch files — is done by the demo that uses it, not here. This
package deliberately has no dependency on `rmf_demos`, since `rmf_demos`
depends on it.

This package ships only `launch/path_guide_adapter.launch.xml`, which brings up
the fleet manager and adapter for a single fleet and takes `config_file` and
`nav_graph_file` as arguments. It deliberately has no dependency on `rmf_demos`,
since `rmf_demos` depends on this package.

## REST API

Routes live under `/open-rmf/rmf_demos_pg/` so this fleet manager can be told
apart from the EasyFullControl one. Paths are kebab-case, query parameters and
JSON keys are snake_case.

| Method | Route | Purpose |
|---|---|---|
| `GET`  | `/open-rmf/rmf_demos_pg/status` | Robot state, optionally for one `robot_name` |
| `POST` | `/open-rmf/rmf_demos_pg/follow-path` | **New.** The whole route in one request |
| `GET`  | `/open-rmf/rmf_demos_pg/stop-robot` | Stop the robot |
| `GET`  | `/open-rmf/rmf_demos_pg/action-paths` | Look up a configured activity path |
| `POST` | `/open-rmf/rmf_demos_pg/start-activity` | Begin an activity, e.g. dock |
| `POST` | `/open-rmf/rmf_demos_pg/toggle-teleop` | Enter or leave teleop mode |
| `POST` | `/open-rmf/rmf_demos_pg/toggle-attach` | Attach or detach a cart |

### `POST /follow-path`

```
POST /open-rmf/rmf_demos_pg/follow-path?robot_name=tinyBot_1&path_id=12
```

```json
{
  "waypoints": [
    {"map_name": "L1", "x": 12.5, "y": 3.2, "yaw": 0.0,
     "speed_limit": null, "arrival_radius": 0.3},
    {"map_name": "L1", "x": 18.1, "y": 3.2, "yaw": 1.57,
     "speed_limit": 0.4, "arrival_radius": 0.3}
  ]
}
```

The manager publishes this as a single `PathRequest` containing the robot's
current location followed by every waypoint, with cumulative per-leg arrival
times.

## Configuration

`config.yaml` at the root of this package is a standalone example, mirroring the
stock `tinyRobot` fleet. A demo world supplies its own via the launch file's
`config_file` argument.

The one key worth reading before you set it is `max_merge_lane_distance`. It is
Path Guide's only tolerance knob and it does double duty: RMF merges the robot
back onto its lane with it, retrying at up to 10x so merging self-heals, and it
is also the upper bound published as `PathWaypoint.merge_radius`. Since the
integrator's arrival test cannot safely exceed that bound, setting it too tight
caps arrival at nothing and paths stall with no such escalation. Size it from a
measured closest approach, not from a cornering parameter.

`skip_rotation_commands` has no effect here. PathGuide never emits a
rotate-in-place waypoint, because such a waypoint is meaningless inside a path
the robot is meant to smooth over — every destination carries the final
orientation for its vertex and the robot owns its own turns. The config parser
logs a notice if the key is present.

## Tests

```bash
colcon test --packages-select rmf_demos_path_guide_adapter
```

`test/test_follow_path_endpoint.py` spins up a real ROS 2 context and asserts on
what actually reaches `robot_path_requests`, rather than mocking the publisher.
