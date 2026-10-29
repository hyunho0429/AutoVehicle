"""Route-region entry gate shared by the planner and dynamic wait guard.

Coordinates are verified against this exact ordered route; segment windows
prevent a nearby parallel/return road from changing the mission identity.
"""
import bisect
import hashlib
import json
import math


def route_fingerprint(points):
    values = [[round(float(p.x), 6), round(float(p.y), 6),
               round(float(getattr(p, 'z', 0.)), 6)] for p in points]
    return hashlib.sha256(json.dumps(values, separators=(',', ':'),
                                     allow_nan=False).encode()).hexdigest()


class RouteAvoidanceGate:
    def __init__(self, points, regions, expected_fingerprint='',
                 max_match_distance_m=12., search_ahead_m=60.,
                 search_behind_m=20., gps_blackout_override=True):
        self.points = list(points)
        if len(self.points) < 2:
            raise ValueError('Route requires at least two points')
        fingerprint = route_fingerprint(self.points)
        if expected_fingerprint and fingerprint != expected_fingerprint:
            raise ValueError('Avoidance route fingerprint mismatch')
        self.s = [0.]
        for a, b in zip(self.points, self.points[1:]):
            distance = math.hypot(b.x-a.x, b.y-a.y)
            if not math.isfinite(distance):
                raise ValueError('Route points must be finite')
            self.s.append(self.s[-1]+distance)
        self.max_distance = float(max_match_distance_m)
        self.ahead, self.behind = float(search_ahead_m), float(search_behind_m)
        if not all(math.isfinite(v) and v > 0 for v in
                   (self.max_distance, self.ahead, self.behind)):
            raise ValueError('Invalid route window')
        self.blackout_override = bool(gps_blackout_override)
        self.progress_s = None
        self.regions = []
        for region in regions:
            start = self._nearest(*region['start_xyz'][:2], 0, len(self.points))
            end = self._nearest(*region['end_xyz'][:2], 0, len(self.points))
            if not start < end:
                raise ValueError('Region endpoints out of route order')
            for key, index in (('start_xyz', start), ('end_xyz', end)):
                x, y = region[key][:2]
                if math.hypot(x-self.points[index].x, y-self.points[index].y) > 3.:
                    raise ValueError('Region endpoint not on this route')
            self.regions.append(dict(region, start_s_m=self.s[start], end_s_m=self.s[end]))

    def _nearest(self, x, y, first, last):
        return min(range(first, last), key=lambda i:
                   (self.points[i].x-x)**2+(self.points[i].y-y)**2)

    def update(self, x, y, gps_blackout=False):
        if not all(isinstance(v, (int, float)) and math.isfinite(v) for v in (x, y)):
            return dict(allowed=False, route_matched=False, regions=[],
                        progress_s_m=self.progress_s, reason='invalid_pose')
        first, last = 0, len(self.points)-1
        if self.progress_s is not None:
            first = max(0, bisect.bisect_left(self.s, self.progress_s-self.behind)-1)
            last = min(last, bisect.bisect_right(self.s, self.progress_s+self.ahead))
        best = None
        for i in range(first, last):
            a, b = self.points[i:i+2]
            dx, dy = b.x-a.x, b.y-a.y
            if dx*dx+dy*dy < 1e-12:
                continue
            t = max(0., min(1., ((x-a.x)*dx+(y-a.y)*dy)/(dx*dx+dy*dy)))
            distance = math.hypot(x-a.x-t*dx, y-a.y-t*dy)
            candidate = distance, self.s[i]+t*(self.s[i+1]-self.s[i])
            if best is None or candidate[0] < best[0]:
                best = candidate
        matched = best is not None and best[0] <= self.max_distance
        if matched:
            self.progress_s = best[1]
        names = [r['name'] for r in self.regions if matched and
                 r['start_s_m'] <= self.progress_s < r['end_s_m']]
        allowed = bool(matched and (names or (gps_blackout and self.blackout_override)))
        return dict(allowed=allowed, route_matched=matched, regions=names,
                    progress_s_m=self.progress_s, gps_blackout=bool(gps_blackout),
                    reason='inside_region' if names else 'gps_blackout' if allowed
                    else 'outside_regions' if matched else 'route_unmatched')
