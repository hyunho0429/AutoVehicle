#!/usr/bin/env python3
"""LiDAR + existing driver's vehicle state -> path/speed decision, never CtrlCmd."""
import json
import math
import sys
import threading
import time
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

import rospy
import yaml
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from purepursuit_mgeo.path import load_mgeo_path
from purepursuit_mgeo.mission_speed_gate import route_digest
from purepursuit_mgeo.obstacle_avoidance import AvoidanceEngine
from purepursuit_mgeo.tunnel_frenet_planner import TrackedObstacle


class TimedInput:
    def __init__(self, timeout=.5):
        self.timeout = timeout
        self.sample = None
        self.last_stamp = None

    def observe(self, row, now, wall):
        stamp = row["timestamp"]
        if type(stamp) not in (int, float) or not math.isfinite(stamp) or stamp <= 0 or not -.02 <= now-stamp <= self.timeout:
            self.sample = None
            return False
        if self.last_stamp is not None and now < self.last_stamp-.02:
            self.sample = None
            self.last_stamp = None
        if self.last_stamp is not None and stamp <= self.last_stamp:
            if stamp == self.last_stamp and self.sample is not None and row != self.sample[2]:
                self.sample = None
            return False
        self.sample = (stamp, wall, row)
        self.last_stamp = stamp
        return True

    def current(self, now, wall):
        if self.sample and -.02 <= now-self.sample[0] <= self.timeout and 0 <= wall-self.sample[1] <= self.timeout:
            return self.sample[2]
        return None


def finite_values(values):
    return all(type(v) in (int, float) and math.isfinite(v) for v in values)


class ObstaclePlannerNode:
    def __init__(self):
        path = rospy.get_param("~path_file")
        self.sha = route_digest(path)
        with open(rospy.get_param("~defaults_file"), encoding="utf-8") as stream:
            self.config = yaml.safe_load(stream)
        self.config["failure_policy"] = rospy.get_param("~failure_policy", self.config["failure_policy"])
        self.engine = AvoidanceEngine(load_mgeo_path(path), self.config,
            front_offset=float(rospy.get_param("~front_reference_offset_m", 3.845)),
            lateral_limit=float(rospy.get_param("~lateral_accel_limit_mps2", 2.)),
            max_curvature=math.tan(float(rospy.get_param("~max_steering_rad", .6981317)))/float(rospy.get_param("~wheelbase_m", 3.)),
            loop=bool(rospy.get_param("~loop_route", True)))
        self.lock = threading.RLock()
        self.vehicle = TimedInput(.5)
        self.obstacles = TimedInput(.6)
        self.gps = None
        self.last_now = None
        self.status_pub = rospy.Publisher("/planning/avoidance/decision", String, queue_size=1)
        rospy.Subscriber("/planning/avoidance/vehicle", String, self.vehicle_callback, queue_size=1)
        rospy.Subscriber("/planning/avoidance/obstacles", String, self.obstacle_callback, queue_size=1)
        rospy.Subscriber("/localization/gps", Odometry, self.gps_callback, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(.2), self.tick)

    def vehicle_callback(self, message):
        with self.lock:
            try:
                row = json.loads(message.data)
                if (row["path_sha256"] != self.sha or row["frame_id"] != "map"
                        or row["speed_unit"] != "m/s" or type(row["vehicle"]) is not list
                        or len(row["vehicle"]) != 4 or not finite_values(row["vehicle"]+[row["progress_s"]])
                        or row["vehicle"][3] < 0 or not 0 <= row["progress_s"] <= self.engine.route.length+.01):
                    raise ValueError("invalid driver vehicle state")
                self.vehicle.observe(row, rospy.Time.now().to_sec(), time.monotonic())
            except (ValueError, TypeError, KeyError):
                self.vehicle.sample = None

    def obstacle_callback(self, message):
        with self.lock:
            try:
                row = json.loads(message.data)
                if row["frame_id"] != "map" or type(row["obstacles"]) is not list:
                    raise ValueError("invalid obstacle frame")
                for o in row["obstacles"]:
                    if (not finite_values([o[k] for k in ("center_x_map", "center_y_map", "length", "width", "velocity_x_map", "velocity_y_map", "yaw")])
                            or o["length"] <= 0 or o["width"] <= 0 or type(o["yaw_valid"]) is not bool):
                        raise ValueError("invalid obstacle geometry")
                self.obstacles.observe(row, rospy.Time.now().to_sec(), time.monotonic())
            except (ValueError, TypeError, KeyError):
                self.obstacles.sample = None

    def gps_callback(self, message):
        with self.lock:
            if message.header.frame_id == "map":
                self.gps = (message.header.stamp.to_sec(), message.pose.pose.position.x, message.pose.pose.position.y)

    def tick(self, _event):
        now, wall = rospy.Time.now().to_sec(), time.monotonic()
        with self.lock:
            if self.last_now is not None and now < self.last_now-.02:
                self.engine.reset()
                self.vehicle = TimedInput(.5)
                self.obstacles = TimedInput(.6)
                self.gps = None
            self.last_now = now
            vehicle = self.vehicle.current(now, wall)
            obstacles = self.obstacles.current(now, wall)
            gps = self.gps
        status = dict(active=False, configured=True, stop_required=True, emergency=False,
                      target_speed=0., points=[], state="WAITING_INPUT", decision_reason="fresh_pose_speed_or_lidar_missing")
        if vehicle is not None and obstacles is not None:
            try:
                status = self.engine.step(tuple(vehicle["vehicle"]), vehicle["progress_s"],
                    [TrackedObstacle.from_dict(o) for o in obstacles["obstacles"]],
                    obstacles["timestamp"], now, gps)
            except Exception as error:
                # Computation faults must not masquerade as a clear road.
                self.engine.reset()
                status["decision_reason"] = "avoidance_planning_error"
                rospy.logerr_throttle(1., "Avoidance planning error: %s", error)
        else:
            self.engine.reset()
        # Stamp the START of calculation. A slow planner cannot revive an old result.
        status.update(timestamp=now, path_sha256=self.sha, frame_id="map", speed_unit="m/s")
        self.status_pub.publish(String(data=json.dumps(status, allow_nan=False)))


if __name__ == "__main__":
    rospy.init_node("merge_ws_obstacle_planner")
    ObstaclePlannerNode()
    rospy.spin()
