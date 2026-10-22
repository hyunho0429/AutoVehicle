"""Atomic, timestamped local path and speed contract for the existing driver."""
import json
import math
from .mission_speed_gate import MissionSpeedGate
from .path import PathPoint, MgeoPurePursuit


class AvoidanceGate(MissionSpeedGate):
    def observe(self, payload, ros_now, wall_now):
        try:
            row = json.loads(payload) if isinstance(payload, str) else dict(payload)
            if row.get("frame_id") != "map":
                raise ValueError("avoidance path frame differs")
            points = row.get("points", [])
            if row.get("active"):
                if not 3 <= len(points) <= 2000 or row["target_speed"] <= 0 or row["target_speed"] > 60/3.6:
                    raise ValueError("invalid avoidance path or speed")
                previous = None
                for p in points:
                    if len(p) != 3 or not all(type(v) in (int, float) and math.isfinite(v) for v in p):
                        raise ValueError("invalid avoidance coordinate")
                    if previous and not 1e-5 < math.hypot(p[0]-previous[0], p[1]-previous[1]) <= 1.:
                        raise ValueError("disconnected avoidance points")
                    previous = p
            elif points:
                raise ValueError("inactive avoidance contains a local path")
        except (TypeError, ValueError, KeyError):
            with self.lock:
                self.sample = None
            return False
        return super().observe(row, ros_now, wall_now)

    def select(self, base_speed, ros_now, wall_now, vehicle, wheelbase, steering_sign):
        with self.lock:
            target_speed, stop, reason = self.constrain(base_speed, ros_now, wall_now)
            if not self.enabled:
                return target_speed, False, None, "avoidance_disabled"
            if stop:
                return 0., True, None, "avoidance_"+reason.replace("roundabout_", "")
            row = self.sample[2]
            if not row["active"]:
                return base_speed, False, None, str(row.get("decision_reason", "avoidance_normal"))
            points = [PathPoint(*p) for p in row["points"]]
            x, y, yaw, speed = vehicle
            # A fresh message must also join the actual vehicle, not a pre-teleport pose.
            nearest = min(range(len(points)), key=lambda i: math.hypot(points[i].x-x, points[i].y-y))
            remaining = sum(math.hypot(b.x-a.x, b.y-a.y) for a, b in zip(points[nearest:], points[nearest+1:]))
            if math.hypot(points[nearest].x-x, points[nearest].y-y) > .946 or remaining < max(1., speed*.5):
                return 0., True, None, "avoidance_path_not_joinable_or_exhausted"
            i = min(nearest, len(points)-2)
            path_yaw = math.atan2(points[i+1].y-points[i].y, points[i+1].x-points[i].x)
            heading_error = math.atan2(math.sin(yaw-path_yaw), math.cos(yaw-path_yaw))
            if abs(heading_error) > math.pi/4:
                return 0., True, None, "avoidance_path_heading_differs"
            controller = MgeoPurePursuit(points, wheelbase, 4., .35, .5, steering_sign)
            steering, goal, point, _, lookahead = controller.compute(x, y, yaw, speed)
            if goal:
                return 0., True, None, "avoidance_horizon_exhausted"
            return target_speed, False, (steering, point, lookahead), str(row.get("decision_reason", "avoidance_path"))
