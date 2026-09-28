"""Pure Pursuit math, kept free of ROS so it can be unit tested.

Everything here works in the odom frame (x forward at spawn, y left, yaw CCW).
"""
import csv
import math


def quaternion_to_yaw(qx, qy, qz, qw):
    """Planar yaw (rotation about z) from a unit quaternion (ZYX convention)."""
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle):
    """Wrap an angle to (-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def load_waypoints(path):
    """Read a CSV with an 'x,y' header (extra columns are ignored)."""
    points = []
    with open(path, newline='') as f:
        reader = csv.DictReader(f)
        for row in reader:
            points.append((float(row['x']), float(row['y'])))
    return points


def save_points(path, points):
    """Write a list of (x, y) or (t, x, y) tuples to CSV."""
    with open(path, 'w', newline='') as f:
        writer = csv.writer(f)
        if points and len(points[0]) == 3:
            writer.writerow(['t', 'x', 'y'])
        else:
            writer.writerow(['x', 'y'])
        for p in points:
            writer.writerow([f'{v:.4f}' for v in p])


class PurePursuit:
    """Geometric path tracker for a rear-axle referenced bicycle model.

    Look-ahead distance is speed adaptive: Ld = ld_min + k * v, clipped to ld_max.
    The target search only moves forward along the path (inside a bounded
    window), so the target index never jumps back or across the track.
    """

    def __init__(self, waypoints, wheelbase=2.7, ld_min=3.0, k=0.4,
                 ld_max=15.0, max_steer=0.5, closed_loop=True,
                 search_window=60):
        if len(waypoints) < 2:
            raise ValueError('need at least 2 waypoints')
        self.path = list(waypoints)
        self.n = len(self.path)
        self.wheelbase = wheelbase
        self.ld_min = ld_min
        self.k = k
        self.ld_max = ld_max
        self.max_steer = max_steer
        self.closed_loop = closed_loop
        self.search_window = search_window
        self.nearest_idx = None

    # ---------- helpers ----------
    def _idx(self, i):
        return i % self.n if self.closed_loop else min(i, self.n - 1)

    @staticmethod
    def _dist(a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def lookahead_distance(self, speed):
        return min(self.ld_max, self.ld_min + self.k * abs(speed))

    def _update_nearest(self, x, y):
        """Nearest waypoint, searched forward from the last one (no index jumps)."""
        if self.nearest_idx is None:
            # first call: global search once
            self.nearest_idx = min(range(self.n),
                                   key=lambda i: self._dist(self.path[i], (x, y)))
            return self.nearest_idx
        best_i = self.nearest_idx
        best_d = self._dist(self.path[self._idx(best_i)], (x, y))
        for step in range(1, self.search_window):
            i = self.nearest_idx + step
            if not self.closed_loop and i >= self.n:
                break
            d = self._dist(self.path[self._idx(i)], (x, y))
            if d < best_d:
                best_d, best_i = d, i
        self.nearest_idx = best_i  # unwrapped counter, keeps growing on loops
        return best_i

    def goal_reached(self, x, y, tol=1.5):
        if self.closed_loop:
            return False
        return (self._idx(self.nearest_idx or 0) >= self.n - 2
                and self._dist(self.path[-1], (x, y)) < tol)

    def cross_track_error(self, x, y):
        """Signed distance to the closest path segment (+ = path is to the left)."""
        i = self._idx(self.nearest_idx if self.nearest_idx is not None else 0)
        best = None
        for j in (i - 1, i):
            if not self.closed_loop and (j < 0 or j + 1 >= self.n):
                continue
            a = self.path[j % self.n]
            b = self.path[(j + 1) % self.n]
            e = _point_segment_signed(x, y, a, b)
            if best is None or abs(e) < abs(best):
                best = e
        return best if best is not None else 0.0

    # ---------- main step ----------
    def compute(self, x, y, yaw, v_cmd, v_meas=None):
        """Return (delta, omega, target_point, ld) for the current state.

        v_cmd is the forward speed we will command; v_meas (odometry) sets the
        adaptive look-ahead. If v_meas is None, v_cmd is used for both.
        """
        ld = self.lookahead_distance(v_cmd if v_meas is None else v_meas)
        start = self._update_nearest(x, y)
        target_i = start
        # walk forward until the point is at least Ld away from the car
        for step in range(self.n):
            i = start + step
            if not self.closed_loop and i >= self.n:
                target_i = self.n - 1
                break
            target_i = i
            if self._dist(self.path[self._idx(i)], (x, y)) >= ld:
                break
        tx, ty = self.path[self._idx(target_i)]

        alpha = normalize_angle(math.atan2(ty - y, tx - x) - yaw)
        # use the real distance to the target (equals Ld except near the end)
        ld_eff = max(self._dist((tx, ty), (x, y)), 1e-3)
        delta = math.atan2(2.0 * self.wheelbase * math.sin(alpha), ld_eff)
        delta = max(-self.max_steer, min(self.max_steer, delta))
        # kinematic bicycle: theta_dot = v * tan(delta) / L
        omega = v_cmd * math.tan(delta) / self.wheelbase
        return delta, omega, (tx, ty), ld_eff


def _point_segment_signed(px, py, a, b):
    ax, ay = a
    bx, by = b
    dx, dy = bx - ax, by - ay
    seg2 = dx * dx + dy * dy
    if seg2 < 1e-12:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / seg2))
    cx, cy = ax + t * dx, ay + t * dy
    dist = math.hypot(px - cx, py - cy)
    cross = dx * (py - ay) - dy * (px - ax)  # > 0 when the car is left of the path
    return -dist if cross > 0 else dist
