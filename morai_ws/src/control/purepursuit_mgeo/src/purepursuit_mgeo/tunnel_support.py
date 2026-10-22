from __future__ import annotations
import math
from dataclasses import replace
from .tunnel_frenet_planner import TrackedObstacle

def stabilize_long_box_yaw(obstacle: TrackedObstacle, cache: dict, now: float) -> TrackedObstacle:
    """Ignore abrupt axis changes on long, thin BBOX tracks.

    A rectangle's yaw and yaw+pi describe the same shape, so compare the
    undirected box axis (modulo pi) before deciding that the tracker flipped.
    """
    if not (obstacle.yaw_valid and math.isfinite(obstacle.yaw)
            and obstacle.yaw_source.upper() == "BBOX"
            and obstacle.length >= 8.0 and obstacle.width <= 1.5):
        return obstacle
    previous = cache.get(obstacle.track_id)
    if previous is not None and now-previous["seen"] > 1.5:
        previous = None
    raw_yaw = obstacle.yaw
    if previous is None:
        cache[obstacle.track_id] = {"yaw": raw_yaw, "seen": now, "locked": False}
        return obstacle
    difference = raw_yaw-previous["yaw"]
    delta = 0.5*math.atan2(math.sin(2.0*difference),math.cos(2.0*difference))
    locked = previous["locked"] or abs(delta) > math.radians(35.0)
    used_yaw = previous["yaw"] if locked else raw_yaw
    cache[obstacle.track_id] = {"yaw": used_yaw, "seen": now, "locked": locked}
    if locked:
        return replace(obstacle,yaw=used_yaw,yaw_source="BBOX_STABILIZED")
    return obstacle


def planned_lateral_acceleration(candidate, vehicle, max_tracking_error):
    if candidate is None or len(candidate.points)<3:
        return 0.0
    x,y=vehicle[:2]
    i=min(range(len(candidate.points)),key=lambda j:math.hypot(candidate.points[j].x-x,candidate.points[j].y-y))
    if math.hypot(candidate.points[i].x-x,candidate.points[i].y-y)>max_tracking_error:
        return 0.0
    i=max(1,min(i,len(candidate.points)-2))
    t=candidate.times;d=candidate.lateral_offsets
    dt0=t[i]-t[i-1];dt1=t[i+1]-t[i]
    if min(dt0,dt1)<=0:
        return 0.0
    value=2.0*((d[i+1]-d[i])/dt1-(d[i]-d[i-1])/dt0)/(dt0+dt1)
    return value if math.isfinite(value) else 0.0


class GpsFreshnessGate:
    """Receipt gap of fresh converted GPS; repeated cached stamps do not count."""

    def __init__(self, now, timeout=1.0, recovery=1.0):
        self.started_at = now
        self.timeout = timeout
        self.recovery = recovery
        self.last_valid_at = None
        self.last_stamp = None
        self.recovery_since = None
        self.blackout = False

    def observe(self, now, stamp, x, y):
        if (not all(math.isfinite(v) for v in (now,stamp,x,y))
                or stamp <= 0.0 or abs(now-stamp) > self.timeout
                or (self.last_stamp is not None and stamp <= self.last_stamp)):
            return False
        if self.last_valid_at is None or now-self.last_valid_at >= self.timeout:
            self.recovery_since = now
        self.last_valid_at = now
        self.last_stamp = stamp
        return True

    def update(self, now):
        origin = self.started_at if self.last_valid_at is None else self.last_valid_at
        if now-origin >= self.timeout:
            self.blackout = True
            self.recovery_since = None
        elif (self.blackout and self.recovery_since is not None
              and now-self.recovery_since >= self.recovery):
            self.blackout = False
        return self.blackout


class RecentThreatMemory:
    """Briefly keep previously relevant objects through a detector dropout."""

    def __init__(self, hold_seconds=1.5):
        self.hold_seconds = hold_seconds
        self.records = {}

    def observe(self, obstacles, now):
        self.records = {k:v for k,v in self.records.items() if 0 <= now-v[0] <= self.hold_seconds}
        for o in obstacles:
            previous = self.records.get(o.track_id)
            self.records[o.track_id] = (now,o,False if previous is None else previous[2])

    def protect(self, track_ids):
        for track_id in track_ids:
            if track_id in self.records:
                t,o,_ = self.records[track_id]
                self.records[track_id] = (t,o,True)

    def supplement(self, obstacles, now, assume_static):
        result=list(obstacles);present={o.track_id for o in obstacles};held=[]
        for track_id,(seen,o,protected) in self.records.items():
            age=now-seen
            if protected and track_id not in present and 0 <= age <= self.hold_seconds:
                if not assume_static:
                    x,y=o.predicted(age);o=replace(o,center_x=x,center_y=y)
                result.append(o);held.append((track_id,round(age,3)))
        return result,held



