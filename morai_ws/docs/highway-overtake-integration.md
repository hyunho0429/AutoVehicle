# Highway overtaking integration

This branch keeps `final_ws_highway_bringup.launch` unchanged and adds
`final_ws_highway_overtake_bringup.launch` as a separate ROS launch. The planner
comes from `whddbs3061/car` branch `highway-overtake-v61` at `347d0c9`.
Only its highway planner, ROS adapter, configuration, launch generator and
focused tests are included.

## Run

Build the `morai_ws` catkin workspace, then launch:

```bash
roslaunch morai_bringup final_ws_highway_overtake_bringup.launch \
  workspace_path:=/opt/AutoVehicle/morai_ws enable_control:=false
```

`enable_control` defaults to `false`. Set it to `true` only for an intended
MORAI drive. The normal launch is still available under its original name.

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
