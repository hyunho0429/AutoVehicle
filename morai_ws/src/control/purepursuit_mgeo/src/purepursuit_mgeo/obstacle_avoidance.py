"""Planning-only adapter for the teammate's tunnel clearance candidate scorer.

No ROS, pedals or lane inference. The original +/-6 m candidate range and
reference fallback policy are explicit configuration, not a collision guarantee.
"""
import math
from dataclasses import replace

from .path import PathPoint
from .tunnel_frenet_planner import FrenetPlanner, ReferencePath
from .tunnel_support import (RecentThreatMemory, GpsFreshnessGate,
                             planned_lateral_acceleration, stabilize_long_box_yaw)


class AvoidanceEngine:
    def __init__(self, points, config, front_offset=3.845, lateral_limit=2.0,
                 max_curvature=.28, loop=True):
        self.route = ReferencePath(points)
        self.config = dict(config)
        self.loop = loop
        self.speed = float(config.get("maneuver_speed_mps", 6.0))
        self.times = [float(t) for t in config.get("planning_times_s", [2., 3., 4.])]
        self.offsets = [float(d) for d in config["avoidance_offsets_m"]]
        self.fallback = config.get("failure_policy", "reference")
        if self.fallback not in ("reference", "stop"):
            raise ValueError("failure_policy must be reference or stop")
        if (not 0 < self.speed <= 60/3.6 or not self.times or not self.offsets
                or not all(math.isfinite(t) and .5 < t <= 5 for t in self.times)
                or not all(math.isfinite(d) and abs(d) <= 6 for d in self.offsets)):
            raise ValueError("invalid avoidance speed, durations or offsets")
        self.geometry = dict(vehicle_length=float(config.get("vehicle_length_m", 4.635)),
            vehicle_width=float(config.get("vehicle_width_m", 1.892)),
            rear_axle_to_center=front_offset-float(config.get("vehicle_length_m", 4.635))/2,
            safety_margin=float(config.get("safety_margin_m", .3)),
            maximum_lateral_acceleration=lateral_limit, maximum_curvature=max_curvature,
            maximum_prediction_time=max(self.times),
            assume_static_obstacles=bool(config.get("assume_static_obstacles", False)))
        if not all(math.isfinite(v) and v > 0 for k, v in self.geometry.items()
                   if k != "assume_static_obstacles"):
            raise ValueError("invalid avoidance geometry or dynamics")
        self.reset()

    def reset(self):
        self.candidate = None
        self.state = "NORMAL"
        self.clear_since = None
        self.last_progress = None
        self.last_obstacle_stamp = None
        self.memory = RecentThreatMemory(float(self.config.get("tunnel_obstacle_hold_s", 1.5)))
        self.gps = None
        self.box_yaws = {}
        self.normalized_obstacles = []
        self.finish_progress = None
        self.direction = 0
        self.tunnel = False

    def _reference(self, progress, speed):
        # Use the controller's branch/progress, including the closed-route seam.
        # A global nearest-point search can select an opposing road at a crossing.
        before = max(0., progress-5.)
        after = progress + max(60., max(speed, self.speed)*max(self.times)+10.)
        if not self.loop:
            after = min(after, self.route.length)
        values = [before + i*.5 for i in range(int((after-before)/.5)+1)]
        values.append(after)
        return ReferencePath([self.route.sample(s % self.route.length if self.loop else s)
                              for s in values])

    def _result(self, reason, stop=False, fallback=False, held=()):
        candidate = self.candidate
        active = candidate is not None
        return dict(state=self.state, decision_reason=reason, active=active,
            stop_required=stop, emergency=False, configured=True,
            target_speed=self.speed if active else 0.,
            reference_fallback=fallback, failure_policy=self.fallback,
            tunnel_mode=self.tunnel, held_obstacles=list(held),
            candidate_offset_m=None if candidate is None else candidate.target_offset,
            candidate_clearance_m=None if candidate is None or not math.isfinite(candidate.minimum_clearance)
                else candidate.minimum_clearance,
            points=[] if candidate is None else [[p.x, p.y, p.z] for p in candidate.points])

    def step(self, vehicle, progress, obstacles, obstacle_stamp, now, gps_observation=None):
        if self.last_progress is not None and (progress < self.last_progress-5.
                                              or abs(progress-self.last_progress) > 20.):
            self.reset()  # New lap/teleport cannot reuse a committed avoidance path.
        self.last_progress = progress
        if self.gps is None:
            self.gps = GpsFreshnessGate(now, self.config.get("tunnel_gps_timeout_s", 1.),
                                       self.config.get("tunnel_gps_recovery_s", 1.))
        if gps_observation is not None:
            self.gps.observe(now, *gps_observation)
        tunnel = self.gps.update(now)
        mode_changed = tunnel != self.tunnel
        self.tunnel = tunnel
        planner = FrenetPlanner(self._reference(progress, vehicle[3]), **self.geometry)
        if tunnel:
            planner.weight_offset = 5.
            planner.weight_safety = 40.
            planner.weight_clearance = 100.
            planner.weight_completion = 10.
        self.planner = planner
        if obstacle_stamp != self.last_obstacle_stamp:
            self.box_yaws = {k: v for k, v in self.box_yaws.items() if 0 <= now-v["seen"] <= 1.5}
            obstacles = [stabilize_long_box_yaw(o, self.box_yaws, now) for o in obstacles]
            self.normalized_obstacles = obstacles
            self.memory.observe(obstacles, obstacle_stamp)
            self.last_obstacle_stamp = obstacle_stamp
        obstacles = self.normalized_obstacles
        # Normalize all boxes to 'now' once, including remembered missing tracks.
        age = max(0., now-obstacle_stamp)
        current = [replace(o, center_x=o.predicted(age)[0], center_y=o.predicted(age)[1])
                   if not planner.assume_static_obstacles else o for o in obstacles]
        current, held = self.memory.supplement(current, now, planner.assume_static_obstacles) if tunnel else (current, [])
        projection = planner.reference.project(vehicle[0], vehicle[1])
        blockers = planner.relevant_obstacles(current, projection.s,
            max(vehicle[3], self.speed), planner.vehicle_width,
            float(self.config.get("trigger_distance_m", 30.)), 3., 0.)
        self.memory.protect([o.track_id for o in blockers])
        if blockers:
            self.clear_since = None
        elif self.clear_since is None:
            self.clear_since = now

        valid = False
        if self.candidate is not None:
            valid, _ = planner.validate_remaining(self.candidate, vehicle, current, 0.,
                                                  planner.vehicle_width/2, self.speed)
        returning = (not blockers and self.candidate is not None
                     and self.clear_since is not None and now-self.clear_since >= 1.5
                     and abs(projection.d-self.candidate.target_offset) <= .3)
        if self.state == "RETURN" and abs(projection.d) <= .15:
            error = math.atan2(math.sin(vehicle[2]-projection.yaw), math.cos(vehicle[2]-projection.yaw))
            if abs(error) <= .08 and not blockers:
                self.candidate = None
                self.state = "NORMAL"
                self.finish_progress = None
                self.direction = 0
                return self._result("reference_path_reacquired", held=held)
        if self.candidate is None and not blockers:
            self.state = "NORMAL"
            return self._result("global_path_tracking", held=held)
        if valid and not mode_changed and not (returning and self.state != "RETURN"):
            return self._result("following_validated_"+self.state.lower(), held=held)

        self.state = "RETURN" if returning or (self.state == "RETURN" and not blockers) else "AVOIDANCE"
        targets = [0.] if self.state == "RETURN" else self.offsets
        acceleration = planned_lateral_acceleration(self.candidate, vehicle, planner.vehicle_width/2) if tunnel else 0.
        budget = ((self.finish_progress-progress)/max(1., vehicle[3], self.speed)
                  if tunnel and self.finish_progress is not None else None)
        previous = self.candidate
        # Same-side comparison before trying the opposite side preserves V1 intent.
        same = [d for d in targets if previous is not None and d*previous.target_offset > 0]
        groups = [same, [d for d in targets if d not in same]] if same else [targets]
        chosen = None
        for group in groups:
            if not group:
                continue
            chosen = planner.plan(*vehicle, self.speed, current, 0., group, self.times,
                initial_lateral_acceleration=acceleration, completion_budget=budget,
                continuation_direction=self.direction if tunnel else 0)
            if chosen is not None:
                break
        if chosen is None and valid:
            self.candidate = previous
            return self._result("retained_checked_path", held=held)
        self.candidate = chosen
        if chosen is None:
            self.finish_progress = None
            self.direction = 0
            self.state = "REFERENCE_FALLBACK" if self.fallback == "reference" else "BLOCKED"
            return self._result("no_collision_free_candidate", stop=self.fallback == "stop",
                                fallback=self.fallback == "reference", held=held)
        movement = chosen.target_offset-projection.d
        if self.finish_progress is None or movement*self.direction < -.15:
            self.direction = 1 if movement > 0 else (-1 if movement < 0 else 0)
            self.finish_progress = progress+max(1., vehicle[3], self.speed)*chosen.planning_time
        return self._result("selected_"+self.state.lower(), held=held)
