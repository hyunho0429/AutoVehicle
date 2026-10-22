#!/usr/bin/env python3
"""Decision-only bridge for merge_ws. Never publishes CtrlCmd or changes steering."""
import json
import math
from pathlib import Path
import sys
import threading
import time

# Run in an existing image before catkin creates this new package's wrapper.
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
import yaml
import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from morai_msgs.msg import EgoVehicleStatus
from lidar_perception.msg import LidarObstacleArray
from purepursuit_mgeo.path import load_mgeo_path
from purepursuit_mgeo.longitudinal_controller import VehicleSpeedSource
from purepursuit_mgeo.mission_speed_gate import route_digest
from roundabout_mission.core import Ego, Decision
from roundabout_mission.regions import load_regions
from roundabout_mission.ros_io import fresh, parse_odom, parse_tracks


class Gate:
    def __init__(self):
        self.lock = threading.RLock()
        self.path_file = rospy.get_param("~path_file")
        self.path_sha = route_digest(self.path_file)
        with open(rospy.get_param("~defaults_file"), encoding="utf-8") as stream:
            self.cfg = yaml.safe_load(stream)
        for key, value in (("ego_front_m", 3.845), ("max_accel", 1.0), ("comfortable_decel", 1.5)):
            self.cfg[key] = float(rospy.get_param("~" + key, value))
        self.enabled = bool(rospy.get_param("~enabled", False))
        self.regions_file = rospy.get_param("~regions_file")
        self.points = load_mgeo_path(self.path_file)
        self.engines = load_regions(self.regions_file, self.points, self.cfg, load_mgeo_path) if self.enabled else []
        if self.enabled and not self.engines:
            raise ValueError("roundabout enabled but region CSV is empty; specify the region first")
        self.received, self.stamps = {}, {}
        self.ego = None
        self.speed = VehicleSpeedSource(timeout_sec=self.cfg["odom_timeout_s"])
        self.last_now = self.last_progress = None
        self.capability = None
        self.frame = self.cfg.get("map_frame", "map")
        self.publisher = rospy.Publisher(rospy.get_param("~status_topic", "/planning/roundabout/status"), String, queue_size=1)
        rospy.Subscriber(rospy.get_param("~odom_topic", "/localization/odometry"), Odometry, self.odom, queue_size=1)
        rospy.Subscriber(rospy.get_param("~vehicle_speed_topic", "/Ego_topic"), EgoVehicleStatus, self.vehicle_speed, queue_size=1)
        if self.enabled:
            rospy.Subscriber("/planning/roundabout/path_capability", String, self.path_capability, queue_size=1)
            rospy.Subscriber(rospy.get_param("~obstacle_topic", "/perception/lidar/tracked_obstacles_map"),
                             LidarObstacleArray, self.obstacles, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.cfg["rate_hz"]), self.tick)

    def receive(self, msg, key, parse):
        with self.lock:
            try:
                stamp = msg.header.stamp.to_sec()
                now, wall = rospy.get_time(), time.monotonic()
                if (not math.isfinite(stamp) or stamp <= 0 or stamp > now + 0.02
                        or now - stamp > self.cfg[key + "_timeout_s"]):
                    raise ValueError("invalid/stale " + key + " timestamp")
                if key in self.stamps and stamp <= self.stamps[key]:
                    return
                value = parse(msg, self.frame)
                if key == "odom":
                    self.ego = value
                else:
                    for _, engine in self.engines:
                        engine.update_tracks(value, stamp)
                self.stamps[key], self.received[key] = stamp, wall
            except (ValueError, TypeError, AttributeError) as exc:
                rospy.logwarn_throttle(2.0, "Roundabout input: %s", str(exc))

    def odom(self, msg):
        self.receive(msg, "odom", parse_odom)

    def obstacles(self, msg):
        self.receive(msg, "obstacle", parse_tracks)

    def vehicle_speed(self, msg):
        with self.lock:
            try:
                value = math.hypot(msg.velocity.x, msg.velocity.y)
                self.speed.observe(value, msg.header.stamp.to_sec(), rospy.get_time(), time.monotonic())
            except (ValueError, TypeError, AttributeError):
                pass

    def reset(self):
        # New lap or simulator-clock epoch: no previous permission survives.
        self.engines = [(name, Decision(engine.path.points, engine.circulating.points, engine.c))
                        for name, engine in self.engines]
        self.stamps.clear()
        self.received.clear()
        self.capability = None

    def path_capability(self, message):
        with self.lock:
            try:
                row = json.loads(message.data)
                stamp, caps = row["timestamp"], row["caps"]
                now, wall = rospy.get_time(), time.monotonic()
                if (row["path_sha256"] != self.path_sha or row["speed_unit"] != "m/s"
                        or not math.isfinite(stamp) or stamp <= 0
                        or not -0.02 <= now-stamp <= self.cfg["decision_timeout_s"]
                        or set(caps) != {name for name, _ in self.engines}
                        or not all(type(value) in (int,float) and math.isfinite(value) and value >= 0
                                   for value in caps.values())):
                    raise ValueError("invalid path capability")
                if self.capability is None or stamp > self.capability[0]:
                    self.capability = (stamp, wall, caps)
            except (ValueError, KeyError, TypeError):
                self.capability = None

    def tick(self, _event):
        with self.lock:
            now, wall = rospy.get_time(), time.monotonic()
            row = dict(timestamp=now, active=False, configured=bool(self.engines),
                       target_speed=0.0, stop_required=False, emergency=False,
                       allowed=False, decision_reason="roundabout_disabled", state="DISABLED")
            if self.enabled:
                if self.last_now is not None and now < self.last_now:
                    self.reset()
                    self.last_progress = None
                progress = None if self.ego is None else self.engines[0][1].path.project(self.ego.x, self.ego.y)[1]
                if (progress is not None and self.last_progress is not None
                        and self.last_progress - progress > 5.0):
                    self.reset()
                self.last_progress = progress
                odom_ok = fresh(self.stamps.get("odom"), self.received.get("odom", 0), now, wall, self.cfg["odom_timeout_s"])
                obstacle_ok = fresh(self.stamps.get("obstacle"), self.received.get("obstacle", 0), now, wall, self.cfg["obstacle_timeout_s"])
                speed = self.speed.sample(now, wall)
                capability_ok = (self.capability is not None and fresh(
                    self.capability[0], self.capability[1], now, wall, self.cfg["decision_timeout_s"]))
                if self.ego is None or not odom_ok or speed is None:
                    row.update(stop_required=True, emergency=True, decision_reason="roundabout_pose_or_speed_stale", state="SAFE_STOP")
                else:
                    ego = Ego(self.ego.x, self.ego.y, self.ego.yaw, speed)
                    rows = []
                    for name, engine in self.engines:
                        _, current_s, _, _ = engine.path.project(ego.x, ego.y)
                        active = engine.m["approach"] <= current_s < engine.m["exit"]
                        cap = self.capability[2][name] if capability_ok else 0.0
                        if active and (not capability_ok or cap < self.cfg["minimum_merge_speed"]):
                            rows.append(dict(timestamp=now, active=True, configured=True, target_speed=0.0,
                                stop_required=True, emergency=False, allowed=False, state="WAIT",
                                decision_reason="roundabout_path_capability_stale_or_too_low", region_id=name))
                            continue
                        if capability_ok and cap >= self.cfg["minimum_merge_speed"]:
                            engine.merge_ceiling = min(engine.c["maximum_merge_speed"], engine.c["test_speed_limit"], cap)
                            engine.merge_target = min(engine.merge_target, engine.merge_ceiling)
                        result = engine.step(ego, now, True, obstacle_ok)
                        result["region_id"] = name
                        rows.append(result)
                    row = next((r for r in rows if r["stop_required"] or r["emergency"]),
                               next((r for r in rows if r["active"]), rows[0]))
                    row.update(configured=True)
                row.update(odom_fresh=odom_ok, obstacle_fresh=obstacle_ok,
                           vehicle_speed_fresh=speed is not None)
            self.last_now = now
            row.update(path_sha256=self.path_sha, speed_unit="m/s", timestamp=now,
                       upstream_version="6.1.0", configured_region_count=len(self.engines))
            self.publisher.publish(String(data=json.dumps(row, allow_nan=False)))


if __name__ == "__main__":
    rospy.init_node("merge_ws_roundabout_gate")
    Gate()
    rospy.spin()
