"""Timestamped roundabout speed ceiling, applied before the existing pedal PI."""
import hashlib
import json
import math
import threading


def route_digest(path):
    with open(path, "rb") as stream:
        return hashlib.sha256(stream.read()).hexdigest()


class MissionSpeedGate:
    def __init__(self, enabled=False, timeout_sec=0.3, path_sha256=""):
        if type(enabled) is not bool or not math.isfinite(timeout_sec) or timeout_sec <= 0:
            raise ValueError("invalid mission gate enable/timeout")
        self.enabled, self.timeout, self.path_sha = enabled, timeout_sec, path_sha256
        self.lock = threading.RLock()
        self.sample = None
        self.last_stamp = None

    def observe(self, payload, ros_now, wall_now):
        with self.lock:
            try:
                row = json.loads(payload) if isinstance(payload, str) else dict(payload)
                stamp = row["timestamp"]
                if (type(stamp) not in (int, float) or not math.isfinite(stamp)
                        or stamp <= 0 or not -0.02 <= ros_now - stamp <= self.timeout):
                    raise ValueError("invalid mission timestamp")
                for key in ("active", "stop_required", "emergency", "configured"):
                    if type(row[key]) is not bool:
                        raise ValueError("invalid mission flag")
                if row.get("speed_unit") != "m/s" or row.get("path_sha256") != self.path_sha:
                    raise ValueError("mission path or speed units differ")
                target = row["target_speed"]
                if type(target) not in (int, float) or not math.isfinite(target) or target < 0:
                    raise ValueError("invalid mission target speed")
                if self.last_stamp is not None and ros_now < self.last_stamp - 0.02:
                    self.sample = None
                    self.last_stamp = None
                if self.last_stamp is not None and stamp <= self.last_stamp:
                    if stamp == self.last_stamp and self.sample is not None and row != self.sample[2]:
                        self.sample = None
                    return False  # A replay cannot keep a decision alive.
                self.last_stamp = stamp
                self.sample = (stamp, wall_now, row)
                return True
            except (ValueError, TypeError, KeyError):
                self.sample = None
                return False

    def constrain(self, base_target_mps, ros_now, wall_now):
        if not self.enabled:
            return base_target_mps, False, "roundabout_disabled"
        with self.lock:
            sample = self.sample
            if sample is None or not (0 <= wall_now - sample[1] <= self.timeout
                                      and -0.02 <= ros_now - sample[0] <= self.timeout):
                return 0.0, True, "roundabout_decision_stale"
            row = sample[2]
            if not row["configured"]:
                return 0.0, True, "roundabout_region_unconfigured"
            if row["stop_required"] or row["emergency"]:
                return 0.0, True, str(row.get("decision_reason", "roundabout_stop"))
            if row["active"]:
                return min(base_target_mps, row["target_speed"]), False, str(row.get("decision_reason", "roundabout_cap"))
            # allowed=False is ordinary cruise outside a configured region.
            return base_target_mps, False, "outside_roundabout_region"
