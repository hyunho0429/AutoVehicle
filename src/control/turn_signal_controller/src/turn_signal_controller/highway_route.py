"""Validate a highway lane path before accepting global-route lateral offset."""
import math


def corridor_matches(points, x, y, yaw, tolerance=1.5):
    if len(points) < 2 or not all(math.isfinite(v) for v in (x, y, yaw, tolerance)):
        return False
    best = None
    for a, b in zip(points, points[1:]):
        if not all(math.isfinite(v) for v in (*a, *b)):
            return False
        dx, dy = b[0]-a[0], b[1]-a[1]
        length2 = dx*dx+dy*dy
        if length2 < 1e-8:
            continue
        t = ((x-a[0])*dx+(y-a[1])*dy)/length2
        # Accept a small gap behind the dynamic path start.
        # All original distance and heading checks remain active.
        if t < 0.0:
            if math.hypot(x-a[0], y-a[1]) > 0.5:
                continue
            t = 0.0
        elif t > 1.0:
            continue
        distance = math.hypot(x-a[0]-t*dx, y-a[1]-t*dy)
        alignment = (math.cos(yaw)*dx+math.sin(yaw)*dy)/math.sqrt(length2)
        if best is None or distance < best[0]:
            best = distance, alignment
    return best is not None and best[0] <= tolerance and best[1] >= 0.5
