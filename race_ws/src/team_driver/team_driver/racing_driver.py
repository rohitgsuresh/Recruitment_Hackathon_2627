#!/usr/bin/env python3
"""Fifth Gear - drivers v2.0 and v2.1: pure pursuit on a path, with a planned speed profile.

v1 (driver.py) is reactive: it only sees what is in front of it. v2 plans ahead.

The path:
  v2.0  follows the track centreline (maps/icra26_centerline.csv).
  v2.1  computes its own racing line from the map at startup:
          a. load the occupancy grid and mark everything closer than
             `clearance` + `line_margin` to a wall as off-limits (this also
             seals the gaps between cones, which are narrower than that);
          b. at every centreline point, measure how far the line may move to
             the left and right while staying in the safe area;
          c. find the offsets that make the whole line bend as little as
             possible (minimum curvature), inside those limits, with L-BFGS-B,
             plus an adjustable penalty on length (`length_weight`), because
             the fastest line lies between the smoothest and the shortest.

Then, for both versions:
  1. Measure the path's curvature (how tightly it bends) at every point.
  2. Plan a speed for every point:
       - corner limit:  v <= sqrt(a_lat / curvature)
       - braking:       slow down early enough to reach the next corner's speed
       - acceleration:  speed can only rise as fast as the car can accelerate
  3. While driving (every LiDAR scan, about 40 Hz): find where we are on the
     path from ground-truth odometry, steer with pure pursuit towards a point
     a lookahead distance ahead, and drive at the planned speed, with a LiDAR
     emergency cap if something is very close directly ahead.

Executables (setup.py):
  driver      -> main()              v2.0, the judged driver
  driver_v21  -> main_racing_line()  v2.1, using the V21_SETTINGS below

    ./scripts/evaluate.sh --team v21_test --driver-exec driver_v21 --laps 3 --headless
"""

import math
import os
import time

import numpy as np
import rclpy
import yaml
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from PIL import Image
from rclpy.node import Node
from scipy.ndimage import distance_transform_edt
from scipy.optimize import minimize
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray

MAX_STEER = 0.4189   # [rad] steering limit of the f1tenth_gym car
WHEELBASE = 0.3302   # [m]   distance between front and rear axles

PATH_CANDIDATES = [
    '/hackathon/maps/icra26_centerline.csv',     # the repository, mounted in the container
    '/opt/hackathon_maps/icra26_centerline.csv',  # fallback copy inside the image
]
MAP_CANDIDATES = [
    '/hackathon/maps/icra26.yaml',
    '/opt/hackathon_maps/icra26.yaml',
]

# Settings for v2.1. Edit these to tune v2.1 without touching v2.0.
V21_SETTINGS = {
    'path_mode': 'racing_line',
    'a_lat': 14.0,
    'v_max': 10.0,
    'a_acc': 8.0,
    'a_brake': 8.0,
    'line_margin': 0.15,
    'length_weight': 0.03,
    'max_cut': 0.0,
    'side_clearance': 0.0,
    'r_tight': 0.0,
    'v_tight': 0.0,
}


# ----------------------------------------------------------------------
# Path helpers (plain numpy, no ROS) - easy to test on their own
# ----------------------------------------------------------------------
def load_path(preferred):
    """Load x, y points from a CSV. Lines starting with # are comments."""
    for p in ([preferred] if preferred else []) + PATH_CANDIDATES:
        if p and os.path.exists(p):
            pts = np.loadtxt(p, delimiter=',', comments='#')[:, :2]
            if np.hypot(*(pts[0] - pts[-1])) < 1e-3:
                pts = pts[:-1]            # closed loop: drop the repeated first point
            return pts, p
    raise FileNotFoundError('No path file found in ' + str(PATH_CANDIDATES))


def smooth_closed(pts, window):
    """Moving average around a closed loop, to remove pixel-tracing jitter."""
    if window <= 1:
        return pts.copy()
    window = window if window % 2 == 1 else window + 1
    half = window // 2
    padded = np.concatenate([pts[-half:], pts, pts[:half]])
    kernel = np.ones(window) / window
    x = np.convolve(padded[:, 0], kernel, mode='valid')
    y = np.convolve(padded[:, 1], kernel, mode='valid')
    return np.stack([x, y], axis=1)


def curvature_closed(pts, k):
    """Signed curvature at each point from the circle through points i-k, i, i+k.

    Curvature is 1 / radius: 0 on a straight, large in a tight corner. Using
    neighbours k points away (rather than 1) averages out small wiggles.
    """
    a = np.roll(pts, k, axis=0)      # point i-k
    c = np.roll(pts, -k, axis=0)     # point i+k
    ab = np.linalg.norm(pts - a, axis=1)
    bc = np.linalg.norm(c - pts, axis=1)
    ca = np.linalg.norm(a - c, axis=1)
    cross = (pts - a)[:, 0] * (c - pts)[:, 1] - (pts - a)[:, 1] * (c - pts)[:, 0]
    return 2.0 * cross / (ab * bc * ca + 1e-9)


def speed_profile(kappa, ds, v_max, a_lat, a_acc, a_brake, v_cap=None):
    """Fastest speed at each point that respects grip, braking and acceleration.

    ds[i] is the distance from point i to point i+1 (around a closed loop).
    v_cap, if given, is an extra per-point speed limit (e.g. for hairpins).
    """
    n = len(kappa)
    v = np.minimum(v_max, np.sqrt(a_lat / np.maximum(np.abs(kappa), 1e-6)))
    if v_cap is not None:
        v = np.minimum(v, v_cap)
    for _ in range(2):   # twice round so the limits wrap across the start line
        for i in range(n - 1, -1, -1):          # backwards: brake before corners
            v[i] = min(v[i], math.sqrt(v[(i + 1) % n] ** 2 + 2.0 * a_brake * ds[i]))
    for _ in range(2):
        for i in range(n):                       # forwards: limited acceleration
            v[i] = min(v[i], math.sqrt(v[i - 1] ** 2 + 2.0 * a_acc * ds[i - 1]))
    return v


# ----------------------------------------------------------------------
# Racing line (v2.1)
# ----------------------------------------------------------------------
def load_map(preferred):
    """Occupancy grid image, resolution [m/px] and origin [m] from a map YAML."""
    for p in ([preferred] if preferred else []) + MAP_CANDIDATES:
        if p and os.path.exists(p):
            with open(p) as f:
                meta = yaml.safe_load(f)
            image_path = meta['image']
            if not os.path.isabs(image_path):
                image_path = os.path.join(os.path.dirname(p), image_path)
            img = np.array(Image.open(image_path).convert('L'))
            origin = (float(meta['origin'][0]), float(meta['origin'][1]))
            return img, float(meta['resolution']), origin, p
    raise FileNotFoundError('No map file found in ' + str(MAP_CANDIDATES))


def normals_closed(pts):
    """Unit vector pointing to the left of the direction of travel at each point."""
    t = np.roll(pts, -1, axis=0) - np.roll(pts, 1, axis=0)
    t /= np.linalg.norm(t, axis=1, keepdims=True) + 1e-9
    return np.stack([-t[:, 1], t[:, 0]], axis=1)


def make_safe_lookup(safe, res, origin):
    """Returns a function telling whether world points (N x 2) are in the safe area."""
    h, w = safe.shape

    def is_safe(xy):
        col = np.floor((xy[:, 0] - origin[0]) / res).astype(int)
        row = h - 1 - np.floor((xy[:, 1] - origin[1]) / res).astype(int)  # image row 0 is the top
        ok = (row >= 0) & (row < h) & (col >= 0) & (col < w)
        out = np.zeros(len(xy), dtype=bool)
        out[ok] = safe[row[ok], col[ok]]
        return out
    return is_safe


def corridor_widths(center, normals, is_safe, max_w=3.0, step=0.02):
    """How far the line may move left and right of each centreline point
    before leaving the safe area. Points not safe themselves get 0 both ways."""
    start_ok = is_safe(center)
    widths = []
    for sign in (1.0, -1.0):                      # left, then right
        w = np.full(len(center), max_w)
        active = start_ok.copy()
        for k in range(1, int(max_w / step) + 1):
            inside = is_safe(center + sign * k * step * normals)
            stopped = active & ~inside
            w[stopped] = (k - 1) * step
            active &= inside
            if not active.any():
                break
        w[~start_ok] = 0.0
        widths.append(w)
    return widths[0], widths[1], start_ok


def min_curvature_offsets(center, normals, w_left, w_right, length_weight=0.0):
    """Sideways offset at each point that minimises the total bending of the line.

    The line is p_i = center_i + alpha_i * normal_i, each alpha_i kept between
    -w_right_i and +w_left_i. We minimise the integral of curvature squared,
    sum(kappa_i^2 * l_i), where l_i is the local point spacing and
    kappa_i = |p_(i+1) - 2 p_i + p_(i-1)| / l_i^2.

    Dividing by the spacing matters: without it, the objective also rewards
    shortening the line, which pulls it to the inside of long corners instead
    of letting it run wide. The gradient is computed exactly (not numerically)
    so the optimiser takes well under a second.

    length_weight adds a penalty on the total length of the line. At 0 the
    result is the pure minimum-curvature line (widest, gentlest corners); larger
    values trade some smoothness for a shorter line. The fastest line is
    somewhere in between, so this is a tuning knob.
    """
    def objective(alpha):
        p = center + alpha[:, None] * normals
        e = np.roll(p, -1, axis=0) - p                  # e_i = p_(i+1) - p_i
        e_len = np.linalg.norm(e, axis=1) + 1e-9
        u = e / e_len[:, None]
        l = 0.5 * (e_len + np.roll(e_len, 1))           # spacing around point i
        d2 = e - np.roll(e, 1, axis=0)                  # p_(i+1) - 2 p_i + p_(i-1)
        d2_sq = np.sum(d2 ** 2, axis=1)
        f = np.sum(d2_sq / l ** 3) + length_weight * np.sum(e_len)

        # d/dp of sum |d2_i|^2 / l_i^3, holding l fixed ...
        wd2 = d2 / (l ** 3)[:, None]
        grad_p = 2.0 * (np.roll(wd2, 1, axis=0) - 2.0 * wd2 + np.roll(wd2, -1, axis=0))
        # ... plus the part that comes from l changing with p
        b = 1.5 * d2_sq / l ** 4
        grad_p += (b + np.roll(b, -1))[:, None] * u - (np.roll(b, 1) + b)[:, None] * np.roll(u, 1, axis=0)
        grad_p += length_weight * (np.roll(u, 1, axis=0) - u)     # d(length)/dp
        return f, np.sum(grad_p * normals, axis=1)

    result = minimize(objective, np.zeros(len(center)), jac=True, method='L-BFGS-B',
                      bounds=list(zip(-w_right, w_left)),
                      options={'maxiter': 5000, 'ftol': 1e-12, 'gtol': 1e-8})
    return result.x, result


def resample_closed(pts, spacing):
    """Evenly spaced points along a closed polyline."""
    closed = np.vstack([pts, pts[:1]])
    seg = np.linalg.norm(np.diff(closed, axis=0), axis=1)
    s = np.concatenate([[0.0], np.cumsum(seg)])
    count = max(int(round(s[-1] / spacing)), 10)
    s_new = np.linspace(0.0, s[-1], count, endpoint=False)
    return np.stack([np.interp(s_new, s, closed[:, 0]),
                     np.interp(s_new, s, closed[:, 1])], axis=1)


def compute_racing_line(center_raw, map_file, clearance, spacing, length_weight=0.0, passes=2):
    img, res, origin, map_source = load_map(map_file)
    free = img >= 128                  # the simulator treats darker than 128 as wall
    dist = distance_transform_edt(free) * res     # metres to the nearest wall pixel
    is_safe = make_safe_lookup(dist >= clearance, res, origin)

    # Each pass optimises around the previous result, re-spaced evenly, so the
    # final line is not distorted by the uneven spacing of the first guess.
    reference = resample_closed(smooth_closed(center_raw, 5), spacing)
    pinned, iterations = 0, 0
    for _ in range(passes):
        normals = normals_closed(reference)
        w_left, w_right, start_ok = corridor_widths(reference, normals, is_safe)
        w_left = np.maximum(w_left - res, 0.0)    # one pixel of margin for rounding
        w_right = np.maximum(w_right - res, 0.0)
        alpha, result = min_curvature_offsets(reference, normals, w_left, w_right, length_weight)
        if pinned == 0:
            pinned = int((~start_ok).sum())
            bounds = (reference + w_left[:, None] * normals,
                      reference - w_right[:, None] * normals)
        iterations += int(result.nit)
        reference = resample_closed(reference + alpha[:, None] * normals, spacing)

    info = {
        'map': map_source,
        'pinned': pinned,
        'iterations': iterations,
        'max_offset': float(np.max(np.min(np.linalg.norm(
            reference[:, None, :] - smooth_closed(center_raw, 5)[None, :, :], axis=2), axis=1))),
        'unsafe': int((~is_safe(reference)).sum()),
        'left': bounds[0],
        'right': bounds[1],
    }
    return reference, info


# ----------------------------------------------------------------------
class RacingDriver(Node):

    def __init__(self, settings=None):
        super().__init__('driver')
        self.settings = settings or {}
        decl = lambda name, default: self.declare_parameter(
            name, self.settings.get(name, default))

        decl('scan_topic', '/scan')
        decl('odom_topic', '/ego_racecar/odom')
        decl('drive_topic', '/drive')
        decl('path_file', '')
        decl('map_file', '')

        # Which path to follow: 'centreline' (v2.0) or 'racing_line' (v2.1)
        decl('path_mode', 'centreline')
        decl('clearance', 0.35)            # [m] from the organisers' README
        decl('line_margin', 0.10)          # [m] extra distance from walls
        decl('line_spacing', 0.2)          # [m] between racing-line points
        decl('length_weight', 0.0)         # 0 = pure minimum curvature; more = shorter line

        # Path processing (centreline only)
        decl('smooth_window', 5)           # points in the moving average
        decl('curvature_offset', 3)        # neighbours used for curvature

        # Speed profile - the main tuning knobs (v2.0 tuned values)
        decl('v_max', 10.0)                # [m/s] top speed
        decl('a_lat', 21.0)                # [m/s^2] cornering (see SUBMISSION.md)
        decl('a_acc', 8.0)                 # [m/s^2] acceleration
        decl('a_brake', 8.0)               # [m/s^2] braking
        decl('speed_scale', 1.0)           # multiply every planned speed
        decl('speed_preview', 2)           # use the speed this many points ahead

        # Pure pursuit
        decl('lookahead_base', 0.6)        # [m]
        decl('lookahead_gain', 0.15)       # [s] extra lookahead per m/s
        decl('lookahead_min', 0.6)         # [m]
        decl('lookahead_max', 2.0)         # [m]
        decl('steer_gain', 1.0)
        decl('max_cut', 0.0)               # [m] max corner-cutting allowed; 0 = off
        decl('lookahead_floor', 0.3)       # [m] shortest lookahead ever used

        # Safety
        decl('emergency_gain', 2.5)        # speed <= gain * distance ahead
        decl('side_clearance', 0.0)        # [m] don't steer towards things this close; 0 = off
        decl('r_tight', 0.0)               # [m] corners tighter than this get v_tight; 0 = off
        decl('v_tight', 0.0)               # [m/s] speed cap in those corners

        p = lambda name: self.get_parameter(name).value
        self.v_max, self.a_lat = p('v_max'), p('a_lat')
        self.a_acc, self.a_brake = p('a_acc'), p('a_brake')
        self.speed_scale, self.speed_preview = p('speed_scale'), p('speed_preview')
        self.la_base, self.la_gain = p('lookahead_base'), p('lookahead_gain')
        self.la_min, self.la_max = p('lookahead_min'), p('lookahead_max')
        self.steer_gain, self.emergency_gain = p('steer_gain'), p('emergency_gain')
        self.smooth_window, self.curv_offset = p('smooth_window'), p('curvature_offset')
        self.max_cut, self.la_floor = p('max_cut'), p('lookahead_floor')
        self.side_clearance = p('side_clearance')
        self.r_tight, self.v_tight = p('r_tight'), p('v_tight')

        raw, source = load_path(p('path_file'))
        self.get_logger().info(f'Loaded {len(raw)} path points from {source}')
        center_length = float(np.sum(np.linalg.norm(np.roll(raw, -1, axis=0) - raw, axis=1)))

        self.smooth_path = True
        self.bounds = None
        if p('path_mode') == 'racing_line':
            try:
                t0 = time.time()
                line, info = compute_racing_line(
                    raw, p('map_file'), p('clearance') + p('line_margin'), p('line_spacing'),
                    p('length_weight'))
                line_length = float(np.sum(np.linalg.norm(np.roll(line, -1, axis=0) - line, axis=1)))
                self.get_logger().info(
                    f'Racing line from {info["map"]}: {line_length:.2f} m '
                    f'(centreline {center_length:.2f} m), max offset {info["max_offset"]:.2f} m, '
                    f'{info["iterations"]} optimiser iterations, {time.time() - t0:.2f} s. '
                    f'Pinned points: {info["pinned"]}. Unsafe points: {info["unsafe"]}.')
                raw = line
                self.smooth_path = False
                self.bounds = (info['left'], info['right'])
            except Exception as e:  # never fail to drive: fall back to the centreline
                self.get_logger().warn(f'Racing line failed ({e}); falling back to the centreline.')

        self.raw = raw
        self.build(raw)

        self.position = None
        self.yaw = 0.0
        self.speed = 0.0
        self.idx = None              # index of the nearest path point
        self.direction_checked = False
        self.tick = 0

        self.drive_pub = self.create_publisher(AckermannDriveStamped, p('drive_topic'), 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/driver/markers', 1)
        # Queue depth 1: always act on the newest message (lesson from v1)
        self.create_subscription(LaserScan, p('scan_topic'), self.scan_callback, 1)
        self.create_subscription(Odometry, p('odom_topic'), self.odom_callback, 1)

    # ------------------------------------------------------------------
    def build(self, raw):
        """Turn raw path points into everything the controller needs."""
        self.pts = smooth_closed(raw, self.smooth_window) if self.smooth_path else raw.copy()
        self.n = len(self.pts)
        self.ds = np.linalg.norm(np.roll(self.pts, -1, axis=0) - self.pts, axis=1)
        self.kappa = curvature_closed(self.pts, self.curv_offset)
        self.cut_window = int(self.la_max / max(float(np.mean(self.ds)), 1e-3)) + 1
        # Tight-corner cap: where the path's radius is below r_tight, never plan
        # faster than v_tight. One a_lat for the whole lap would otherwise be
        # set by the single tightest hairpin.
        v_cap = None
        if self.r_tight > 0 and self.v_tight > 0:
            tight = np.abs(self.kappa) >= 1.0 / self.r_tight
            v_cap = np.where(tight, self.v_tight, np.inf)
            self.log_tight_segments(tight)
        self.v_plan = speed_profile(self.kappa, self.ds, self.v_max,
                                    self.a_lat, self.a_acc, self.a_brake, v_cap)
        v_mid = 0.5 * (self.v_plan + np.roll(self.v_plan, -1))
        predicted = float(np.sum(self.ds / np.maximum(v_mid, 0.1)))
        self.get_logger().info(
            f'Path: {self.n} points, {self.ds.sum():.2f} m. '
            f'Planned speed {self.v_plan.min():.2f} to {self.v_plan.max():.2f} m/s. '
            f'Predicted lap {predicted:.2f} s (ideal, before tracking losses).')

    def log_tight_segments(self, tight):
        """Log where the tight-corner cap applies, one line per corner."""
        if not tight.any():
            self.get_logger().info('Tight-corner cap: no corners tighter than '
                                   f'{self.r_tight:.2f} m radius.')
            return
        idx = np.nonzero(tight)[0]
        groups, start = [], idx[0]
        for a, b in zip(idx[:-1], idx[1:]):
            if b != a + 1:
                groups.append((start, a))
                start = b
        groups.append((start, idx[-1]))
        if len(groups) > 1 and groups[0][0] == 0 and groups[-1][1] == self.n - 1:
            groups[0] = (groups[-1][0] - self.n, groups[0][1])   # wraps past the start
            groups.pop()
        for g0, g1 in groups:
            seg = np.arange(g0, g1 + 1) % self.n
            mid = self.pts[seg[len(seg) // 2]]
            r_min = 1.0 / float(np.max(np.abs(self.kappa[seg])))
            self.get_logger().info(
                f'Tight-corner cap {self.v_tight:.2f} m/s: {len(seg)} points around '
                f'({mid[0]:.2f}, {mid[1]:.2f}), tightest radius {r_min:.2f} m.')

    # ------------------------------------------------------------------
    def odom_callback(self, msg):
        self.position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        q = msg.pose.pose.orientation
        self.yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                              1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.speed = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)

        if not self.direction_checked:
            # The path may run clockwise or anticlockwise. Make sure it runs
            # the same way the car is facing at the start.
            i = self.nearest_index(*self.position, global_search=True)
            tangent = self.pts[(i + 1) % self.n] - self.pts[i]
            if math.cos(math.atan2(tangent[1], tangent[0]) - self.yaw) < 0:
                self.get_logger().info('Path runs against the car: reversing it.')
                self.raw = self.raw[::-1].copy()
                self.build(self.raw)
            self.direction_checked = True

    def nearest_index(self, x, y, global_search=False):
        """Closest path point. Searches just around the last one, so the car
        never 'jumps' to a different part of the track that happens to be close
        (the infield hairpins run close to each other)."""
        if global_search or self.idx is None:
            d = np.hypot(self.pts[:, 0] - x, self.pts[:, 1] - y)
            return int(np.argmin(d))
        window = (self.idx + np.arange(-5, 40)) % self.n
        d = np.hypot(self.pts[window, 0] - x, self.pts[window, 1] - y)
        if d.min() > 2.0:          # lost the path somehow: search everywhere
            return self.nearest_index(x, y, global_search=True)
        return int(window[np.argmin(d)])

    # ------------------------------------------------------------------
    def scan_callback(self, scan):
        if self.position is None or not self.direction_checked:
            self.publish(0.0, 0.0)
            return

        x, y = self.position
        self.idx = self.nearest_index(x, y)

        # Pure pursuit: walk along the path until we are `lookahead` metres ahead
        lookahead = float(np.clip(self.la_base + self.la_gain * self.speed,
                                  self.la_min, self.la_max))

        # Aiming at a point L ahead on a curve of radius R cuts inside the curve
        # by about L^2 / (8 R) (the sagitta). In a tight hairpin a normal
        # lookahead points across the wall tip, so limit L to keep the cut
        # below max_cut, using the tightest curvature coming up.
        if self.max_cut > 0:
            ahead = (self.idx + np.arange(self.cut_window)) % self.n
            k = float(np.max(np.abs(self.kappa[ahead])))
            limit = math.sqrt(8.0 * self.max_cut / max(k, 1e-6))
            lookahead = min(lookahead, max(limit, self.la_floor))
        j, travelled = self.idx, 0.0
        for _ in range(self.n):
            if travelled >= lookahead:
                break
            travelled += self.ds[j]
            j = (j + 1) % self.n
        tx, ty = self.pts[j]

        # Target in the car's own frame (x forward, y left)
        dx, dy = tx - x, ty - y
        local_x = math.cos(self.yaw) * dx + math.sin(self.yaw) * dy
        local_y = -math.sin(self.yaw) * dx + math.cos(self.yaw) * dy
        dist2 = max(local_x ** 2 + local_y ** 2, 1e-6)
        arc_curvature = 2.0 * local_y / dist2          # the arc through the target
        steering = math.atan(WHEELBASE * arc_curvature) * self.steer_gain
        steering = float(np.clip(steering, -MAX_STEER, MAX_STEER))

        ranges = np.nan_to_num(np.asarray(scan.ranges), nan=10.0, posinf=10.0)
        ranges[ranges <= 0.0] = 10.0     # 0 means no return, not a wall at 0 m
        angles = scan.angle_min + np.arange(len(ranges)) * scan.angle_increment

        # Side check (from v1): never steer towards something that is already
        # close beside the car. In the tight hairpin this stops the car turning
        # in onto the wall tip; it straightens for a moment and rejoins the line.
        if self.side_clearance > 0:
            left = (angles > math.radians(30)) & (angles < math.radians(110))
            right = (angles < -math.radians(30)) & (angles > -math.radians(110))
            if steering > 0 and np.min(ranges[left]) < self.side_clearance:
                steering = 0.0
            elif steering < 0 and np.min(ranges[right]) < self.side_clearance:
                steering = 0.0

        # Planned speed, looked up slightly ahead to allow for reaction delay
        speed = self.v_plan[(self.idx + self.speed_preview) % self.n] * self.speed_scale

        # Emergency cap from the LiDAR: never drive fast at something close ahead
        if self.emergency_gain > 0:
            front = float(np.min(ranges[np.abs(angles) < math.radians(5)]))
            speed = min(speed, self.emergency_gain * front)
        speed = max(speed, 0.5)          # never stop completely (15 s stuck = DNF)

        self.publish(steering, speed)

        self.tick += 1
        if self.tick % 40 == 0:
            self.publish_path_markers()
        if self.tick % 5 == 0:
            self.publish_target_marker(tx, ty)

    # ------------------------------------------------------------------
    def publish(self, steering, speed):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.steering_angle = float(steering)
        msg.drive.speed = float(speed)
        self.drive_pub.publish(msg)

    def line_marker(self, pts, ns, marker_id, width, colors=None, rgb=(0.6, 0.6, 0.6)):
        m = Marker()
        m.header.frame_id = 'map'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns, m.id = ns, marker_id
        m.type, m.action = Marker.LINE_STRIP, Marker.ADD
        m.scale.x = width
        m.pose.orientation.w = 1.0
        m.color.r, m.color.g, m.color.b, m.color.a = rgb[0], rgb[1], rgb[2], 1.0
        for i in list(range(len(pts))) + [0]:
            m.points.append(Point(x=float(pts[i, 0]), y=float(pts[i, 1]), z=0.05))
            if colors is not None:
                m.colors.append(colors[i])
        return m

    def publish_path_markers(self):
        """Draw the path in RViz, coloured by planned speed (red slow, green fast),
        plus the safe-corridor limits in grey when using the racing line."""
        top = max(float(self.v_plan.max()), 1e-3)
        colors = []
        for i in range(self.n):
            f = float(self.v_plan[i]) / top
            c = Marker().color
            c.r, c.g, c.b, c.a = 1.0 - f, f, 0.2, 1.0
            colors.append(c)
        arr = MarkerArray()
        arr.markers.append(self.line_marker(self.pts, 'path', 0, 0.05, colors))
        if self.bounds is not None:
            arr.markers.append(self.line_marker(self.bounds[0], 'bounds', 2, 0.02))
            arr.markers.append(self.line_marker(self.bounds[1], 'bounds', 3, 0.02))
        self.marker_pub.publish(arr)

    def publish_target_marker(self, tx, ty):
        m = Marker()
        m.header.frame_id = 'map'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns, m.id = 'target', 1
        m.type, m.action = Marker.SPHERE, Marker.ADD
        m.pose.position.x, m.pose.position.y, m.pose.position.z = float(tx), float(ty), 0.1
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.2
        m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.8, 0.0, 1.0
        arr = MarkerArray()
        arr.markers.append(m)
        self.marker_pub.publish(arr)


def run(settings, args=None):
    rclpy.init(args=args)
    node = RacingDriver(settings)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main(args=None):
    """v2.0: centreline. This is the judged driver (`ros2 run team_driver driver`)."""
    run({}, args)


def main_racing_line(args=None):
    """v2.1: our own racing line, computed from the map at startup."""
    run(V21_SETTINGS, args)


if __name__ == '__main__':
    main()
