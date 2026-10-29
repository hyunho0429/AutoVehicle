# Highway overtaking integration

This branch keeps `final_ws_highway_bringup.launch` unchanged and adds
`final_ws_highway_overtake_bringup.launch` as a separate ROS launch. The planner
comes from `whddbs3061/car` branch `highway-overtake-v61` at `347d0c9`.
Only its highway planner, ROS adapter, configuration, launch generator and
focused tests are included.

## One-line launch

After building the `morai_ws` catkin workspace and sourcing `devel/setup.bash`,
start perception and planning without sending vehicle control:

```bash
roslaunch morai_bringup merged_highway.launch
```

For an intended MORAI drive, pass the MORAI PC address and explicitly enable
control. The wrapper starts at conservative 5.0/7.2 km/h highway speeds:

```bash
roslaunch morai_bringup merged_highway.launch morai_host_ip:=192.168.0.147 enable_control:=true
```

The address can instead come from `MORAI_IP`. `enable_control` deliberately
defaults to `false`. The original launch remains available under its original
name. `merged_highway.launch` disables the unrelated roundabout merge gate and
resolves the workspace path from the installed `morai_bringup` package.

The wrapper follows the competition v1.2 vehicle-network ports: Ctrl Cmd
9094 -> 9093, CollisionData 9091 -> 9092, and Competition Vehicle Status
9088 -> 9099. It does not start Ego Vehicle Status. The longitudinal controller
requires fresh Competition `vx`; missing or stale status produces a full-brake
command. GPS, IMU, camera and VLP16 remain as the separately allowed sensor
UDP inputs described by rules 2-8 items 4-7. See
`docs/COMPETITION_NETWORK_CONTRACT_KO.md` for the audit boundary and live-capture
verification requirement.

## Interface

- Inputs: `/avoidance_path_manager/trajectory`, `/localization/odometry`,
  `/perception/lidar/tracked_obstacles_map`, `/perception/camera/lane_info`,
  `/planning/highway_route_active`, and
  `/planning/highway_lane_change_request`.
- The overtaking adapter replaces the `highway_lane_strategy` node in this
  launch. It retains `/highway_lane_strategy/trajectory`, `active_path`,
  `stop_required`, `target_speed_mps`, and `state` for the existing controller
  and route gate. It does not publish a vehicle command.
- The existing adaptive curvature controller remains the sole publisher of
  `/control/ctrl_cmd` in this launch. The existing downstream safety and MORAI
  command path are unchanged.

The separate mission region file only enables highway handoff at the end of
the overtaking zone. Recreate the generated launch and region file after
changing the base highway launch or route with:

```bash
python3 docker/final_ws/make_overtake_launch.py
python3 docker/final_ws/make_overtake_launch.py --check
```

The planner predicts front/rear bumper gaps and closing speeds from tracked
LiDAR vehicles before changing lanes. This integration uses 6 m front and 7 m
rear minimum gaps and requires two clear planning ticks. A stale LiDAR stream
or base trajectory produces a stop during an active overtaking plan. The
offline tests and simulated traffic scenarios do not establish MORAI driving
performance; validate sensor calibration and lane transitions in MORAI before
enabling command output.
