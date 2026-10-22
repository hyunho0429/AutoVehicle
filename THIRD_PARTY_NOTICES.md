# Third-party source attribution

## Tunnel clearance V1

Source: [x010806-sudo/morai-tunnel-clearance](https://github.com/x010806-sudo/morai-tunnel-clearance/tree/b2aace7baa3e19ea026baae2dc0bd30e72594fac).
Pinned commit: `b2aace7baa3e19ea026baae2dc0bd30e72594fac`.
Copyright remains with the upstream contributors. No explicit license file or
license declaration was present in the supplied repository at this revision;
the MIT notice below applies to the separately attributed roundabout source.

Imported `frenet_planner.py` as `purepursuit_mgeo/tunnel_frenet_planner.py`;
extracted numerical/memory/GPS helpers from `frenet_fsm_node.py` into
`tunnel_support.py`; and adapted `lidar_tracked_obstacle_state_node.py` as
`scripts/merge_ws_obstacle_states.py`. Numerical tests derive from the upstream
`test_tunnel_clearance.py`. The original standalone FSM/command publisher and
host apply/restore scripts are not installed or run.

Integration adds a decision-only planner adapter, a single atomic path/speed
contract, the existing controller's actual speed and route progress, closed-loop
route support, shared LiDAR transport, current vehicle geometry/dynamics, source
timestamps/frame validation and dual-clock timeouts. The upstream reference
fallback policy and candidate offsets/costs are retained by default. General-road
moving obstacles use measured velocities rather than the donor fixed-scene
launch's forced-static assumption. See README for the reference fallback limits.

## Roundabout V6.1

Decision model imported from the `roundabout_v6_1/roundabout_v6_1` directory of
[ohdang23/yujin_morai](https://github.com/ohdang23/yujin_morai/tree/6c7d805c993f8f2f0d73362de1898206401e0b05/roundabout_v6_1/roundabout_v6_1).
Upstream commit: `6c7d805c993f8f2f0d73362de1898206401e0b05`.
The upstream `roundabout_mission/package.xml` declares the MIT license.
Copyright in the original decision/prediction code remains with its upstream contributors.

Imported modules: `core.py`, `entry_decision.py`, `prediction.py`, `ros_io.py`,
`visualization.py`, and `__init__.py` under `roundabout_mission`.
Decision tuning derives from the upstream V6.1 speed-test configuration. The
standalone pedal driver, host installers, hard-coded mission geometry, historic
documents and recorded trial artifacts are not part of the active integration.

Integration changes: explicit CSV-only region selection; replacement of the
hard-coded stop/exit indices; present-overlap protection independent of the rear
TTC exception; a decision-only ROS adapter; authoritative Ego vehicle speed;
path digest and dual-clock watchdogs; feasible region speed ceilings from the
existing planner; and integration before the existing pedal PI with a single
final command publisher. The original gap/occlusion/relative-following model
otherwise remains the donor's V6.1 implementation.

## MIT License

Copyright (c) the original contributors to ohdang23/yujin_morai

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
IN THE SOFTWARE.
