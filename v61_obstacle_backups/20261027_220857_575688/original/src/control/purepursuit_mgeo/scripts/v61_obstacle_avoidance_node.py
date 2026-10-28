#!/usr/bin/env python3
"""Tunnel-clearance planning only; v61 retains all vehicle command authority."""
import copy
import json
import math

import rospy
from nav_msgs.msg import Path
from std_msgs.msg import Bool, String
from purepursuit_mgeo.tunnel_clearance_fsm import (
    FrenetFsmNode, NORMAL, AVOIDANCE, RETURN, RecentThreatMemory,
)


def fresh(now, stamp, timeout):
    return stamp is not None and math.isfinite(stamp) and -0.05 <= now-stamp <= timeout


def mission_owner(now, highway, roundabout, timeout=0.7):
    """Unknown ownership must never start an independent avoidance maneuver."""
    for name, sample in (("highway", highway), ("roundabout", roundabout)):
        if sample is None or not fresh(now, sample["timestamp"], timeout):
            return "mission_status_unavailable"
        if sample["active"]:
            return name
    return None


class V61ObstacleAvoidance(FrenetFsmNode):
    def __init__(self):
        self.ready = False
        self.base = self.base_at = self.base_stop_at = None
        self.base_stop = True
        self.highway = self.roundabout = None
        self.was_suspended = True
        super().__init__()
        if self.enable_control:
            raise ValueError("This adapter must run with enable_control=false")
        self.output_path = rospy.Publisher("/integration/avoidance/base_path", Path, queue_size=1)
        self.output_stop = rospy.Publisher("/integration/avoidance/base_stop", Bool, queue_size=1)
        self.output_active = rospy.Publisher("/integration/avoidance/active", Bool, queue_size=1)
        self.output_status = rospy.Publisher("/integration/avoidance/status", String, queue_size=1)
        rospy.Subscriber("/integration/base_path", Path, self._base_cb, queue_size=1)
        rospy.Subscriber("/integration/base_stop", Bool, self._base_stop_cb, queue_size=1)
        rospy.Subscriber("/highway_lane_strategy/state", String,
                         lambda m: self._mission_cb(m, "highway"), queue_size=1)
        rospy.Subscriber("/roundabout_merge_gate/status", String,
                         lambda m: self._mission_cb(m, "roundabout"), queue_size=1)
        self.ready = True

    def _base_cb(self, msg):
        now = rospy.get_time()
        if (msg.header.frame_id != self.map_frame or len(msg.poses) < 2
                or not fresh(now, msg.header.stamp.to_sec(), 0.5)
                or any(not all(math.isfinite(v) for v in
                    (p.pose.position.x, p.pose.position.y, p.pose.position.z))
                    for p in msg.poses)):
            return
        with self.lock:
            self.base, self.base_at = msg, now

    def _base_stop_cb(self, msg):
        with self.lock:
            self.base_stop, self.base_stop_at = bool(msg.data), rospy.get_time()

    def _mission_cb(self, msg, name):
        try:
            value = json.loads(msg.data)
            if type(value["active"]) is not bool or not fresh(
                    rospy.get_time(), float(value["timestamp"]), 0.7):
                return
            value["timestamp"] = float(value["timestamp"])
            with self.lock:
                old = getattr(self, name)
                if old is None or value["timestamp"] > old["timestamp"]:
                    setattr(self, name, value)
        except (ValueError, TypeError, KeyError):
            return

    def _reset_maneuver(self):
        self._transition(NORMAL, "v61 mission owns path")
        self.reference_fallback_active = False
        self._last_published_path = None
        self.path_stamp = 0.0
        self.last_plan_at = float('-inf')
        self.threat_memory = RecentThreatMemory(self.threat_memory.hold_seconds)
        self._held_obstacles = []

    def _output(self, path, stop, active, reason):
        # A kept, recently revalidated candidate may have an old creation stamp.
        # Preserve its geometry/seq but stamp this execution heartbeat afresh.
        if path is not None:
            path = copy.deepcopy(path)
            path.header.stamp = rospy.Time.now()
            self.output_path.publish(path)
        self.output_stop.publish(Bool(data=bool(stop)))
        self.output_active.publish(Bool(data=bool(active)))
        self.output_status.publish(String(data=json.dumps({
            "timestamp": rospy.get_time(), "active": bool(active),
            "state": self.state, "source": reason, "stop": bool(stop),
            "tunnel_mode": self.tunnel_mode,
            "blocker_count": len(self.planner.last_blocker_diagnostics),
        }, separators=(",", ":"))))

    def _control_callback(self, event):
        if not self.ready:
            return
        with self.lock:
            now = rospy.get_time()
            base_ok = (self.base is not None and fresh(now, self.base_at, 0.5)
                       and fresh(now, self.base.header.stamp.to_sec(), 0.5))
            odom = self.latest_odom
            odom_ok = (odom is not None and odom.header.frame_id == self.map_frame
                       and fresh(now, self.odom_received_at, self.odom_stale_timeout)
                       and fresh(now, odom.header.stamp.to_sec(), self.odom_stale_timeout)
                       and all(math.isfinite(v) for v in self._vehicle_state(odom)))
            stopped = (not base_ok or not odom_ok or self.base_stop
                       or not fresh(now, self.base_stop_at, 0.5))
            owner = mission_owner(now, self.highway, self.roundabout)
            if stopped or owner is not None:
                if not self.was_suspended:
                    self._reset_maneuver()
                self.was_suspended = True
                self._update_tunnel_mode(now)
                self._output(self.base if base_ok else None, stopped, False,
                             "v61_input_stop" if stopped else owner)
                return
            if self.was_suspended:
                self._reset_maneuver()
                self.was_suspended = False
            super()._control_callback(event)

    def _follow(self, controller, vehicle, desired_speed, blocker_count):
        # Only preview steering/state. The inherited control publisher is disabled.
        ended = super()._follow(controller, vehicle, desired_speed, blocker_count)
        active = (self.state in (AVOIDANCE, RETURN)
                  and self.active_candidate is not None and self.local_controller is not None)
        self._output(self._last_published_path if active else self.base,
                     False, active, "frenet_"+self.state.lower() if active else "v61_reference")
        return ended


if __name__ == "__main__":
    V61ObstacleAvoidance()
    rospy.spin()
