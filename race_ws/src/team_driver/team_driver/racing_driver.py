#!/usr/bin/env python3
"""Fifth Gear - driver v2.0: pure pursuit on a path, with a planned speed profile.

v1 (driver.py) is reactive: it only sees what is in front of it. v2 plans ahead.

At startup:
  1. Load a path around the track. For v2.0 this is the centreline in
     maps/icra26_centerline.csv (generated from the map by the organisers'
     track_tool.py). v2.1 will replace it with our own racing line.
  2. Smooth it and measure its curvature (how tightly it bends) at every point.
  3. Plan a speed for every point:
       - corner limit:  v <= sqrt(a_lat / curvature)   (grip limit in turns)
       - braking:       slow down early enough to reach the next corner's speed
       - acceleration:  speed can only rise as fast as the car can accelerate

While driving (every LiDAR scan, about 40 Hz):
  4. Find where we are on the path using ground-truth odometry.
  5. Pure pursuit: pick a target point a lookahead distance ahead on the path
     and steer along the circular arc that reaches it.
  6. Drive at the planned speed, with a LiDAR emergency cap if something is
     very close directly ahead.

Run it next to v1 with:
    ./scripts/evaluate.sh --team v2_test --driver-exec driver_v2 --laps 3 --headless
"""

import math
import os

import numpy as np
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray

MAX_STEER = 0.4189   # [rad] steering limit of the f1tenth_gym car
WHEELBASE = 0.3302   # [m]   distance between front and rear axles

PATH_CANDIDATES = [
    '/hackathon/maps/icra26_centerline.csv',     # the repository, mounted in the container
    '/opt/hackathon_maps/icra26_centerline.csv',  # fallback copy inside the image
]


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
        return pts
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


def speed_profile(kappa, ds, v_max, a_lat, a_acc, a_brake):
    """Fastest speed at each point that respects grip, braking and acceleration.

    ds[i] is the distance from point i to point i+1 (around a closed loop).
    """
    n = len(kappa)
    v = np.minimum(v_max, np.sqrt(a_lat / np.maximum(np.abs(kappa), 1e-6)))
    for _ in range(2):   # twice round so the limits wrap across the start line
        for i in range(n - 1, -1, -1):          # backwards: brake before corners
            v[i] = min(v[i], math.sqrt(v[(i + 1) % n] ** 2 + 2.0 * a_brake * ds[i]))
    for _ in range(2):
        for i in range(n):                       # forwards: limited acceleration
            v[i] = min(v[i], math.sqrt(v[i - 1] ** 2 + 2.0 * a_acc * ds[i - 1]))
    return v


# ----------------------------------------------------------------------
class RacingDriver(Node):

    def __init__(self):
        super().__init__('driver')

        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('odom_topic', '/ego_racecar/odom')
        self.declare_parameter('drive_topic', '/drive')
        self.declare_parameter('path_file', '')

        # Path processing
        self.declare_parameter('smooth_window', 5)        # points in the moving average
        self.declare_parameter('curvature_offset', 3)     # neighbours used for curvature

        # Speed profile - the main tuning knobs
        self.declare_parameter('v_max', 8.0)              # [m/s] top speed
        self.declare_parameter('a_lat', 17.0)              # [m/s^2] cornering grip
        self.declare_parameter('a_acc', 5.0)              # [m/s^2] acceleration
        self.declare_parameter('a_brake', 5.0)            # [m/s^2] braking
        self.declare_parameter('speed_scale', 1.0)        # multiply every planned speed
        self.declare_parameter('speed_preview', 2)        # use the speed this many points ahead

        # Pure pursuit
        self.declare_parameter('lookahead_base', 0.6)     # [m]
        self.declare_parameter('lookahead_gain', 0.15)    # [s] extra lookahead per m/s
        self.declare_parameter('lookahead_min', 0.6)      # [m]
        self.declare_parameter('lookahead_max', 2.0)      # [m]
        self.declare_parameter('steer_gain', 1.0)

        # Safety
        self.declare_parameter('emergency_gain', 2.5)     # speed <= gain * distance ahead

        p = lambda name: self.get_parameter(name).value
        self.v_max, self.a_lat = p('v_max'), p('a_lat')
        self.a_acc, self.a_brake = p('a_acc'), p('a_brake')
        self.speed_scale, self.speed_preview = p('speed_scale'), p('speed_preview')
        self.la_base, self.la_gain = p('lookahead_base'), p('lookahead_gain')
        self.la_min, self.la_max = p('lookahead_min'), p('lookahead_max')
        self.steer_gain, self.emergency_gain = p('steer_gain'), p('emergency_gain')
        self.smooth_window, self.curv_offset = p('smooth_window'), p('curvature_offset')

        raw, source = load_path(p('path_file'))
        self.get_logger().info(f'Loaded {len(raw)} path points from {source}')
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
        self.pts = smooth_closed(raw, self.smooth_window)
        self.n = len(self.pts)
        self.ds = np.linalg.norm(np.roll(self.pts, -1, axis=0) - self.pts, axis=1)
        self.kappa = curvature_closed(self.pts, self.curv_offset)
        self.v_plan = speed_profile(self.kappa, self.ds, self.v_max,
                                    self.a_lat, self.a_acc, self.a_brake)
        v_mid = 0.5 * (self.v_plan + np.roll(self.v_plan, -1))
        predicted = float(np.sum(self.ds / np.maximum(v_mid, 0.1)))
        self.get_logger().info(
            f'Path: {self.n} points, {self.ds.sum():.2f} m. '
            f'Planned speed {self.v_plan.min():.2f} to {self.v_plan.max():.2f} m/s. '
            f'Predicted lap {predicted:.2f} s (ideal, before tracking losses).')

    # ------------------------------------------------------------------
    def odom_callback(self, msg):
        self.position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        q = msg.pose.pose.orientation
        self.yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                              1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.speed = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)

        if not self.direction_checked:
            # The CSV may list points clockwise or anticlockwise. Make sure the
            # path runs the same way the car is facing at the start.
            i = self.nearest_index(*self.position, global_search=True)
            tangent = self.pts[(i + 1) % self.n] - self.pts[i]
            if math.cos(math.atan2(tangent[1], tangent[0]) - self.yaw) < 0:
                self.get_logger().info('Path runs against the car: reversing it.')
                self.build(self.pts[::-1].copy())
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

        # Planned speed, looked up slightly ahead to allow for reaction delay
        speed = self.v_plan[(self.idx + self.speed_preview) % self.n] * self.speed_scale

        # Emergency cap from the LiDAR: never drive fast at something close ahead
        if self.emergency_gain > 0:
            ranges = np.nan_to_num(np.asarray(scan.ranges), nan=10.0, posinf=10.0)
            angles = scan.angle_min + np.arange(len(ranges)) * scan.angle_increment
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

    def publish_path_markers(self):
        """Draw the path in RViz, coloured by planned speed (red slow, green fast)."""
        m = Marker()
        m.header.frame_id = 'map'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns, m.id = 'path', 0
        m.type, m.action = Marker.LINE_STRIP, Marker.ADD
        m.scale.x = 0.05
        m.pose.orientation.w = 1.0
        top = max(float(self.v_plan.max()), 1e-3)
        for i in list(range(self.n)) + [0]:
            m.points.append(Point(x=float(self.pts[i, 0]), y=float(self.pts[i, 1]), z=0.05))
            f = float(self.v_plan[i]) / top
            c = Marker().color
            c.r, c.g, c.b, c.a = 1.0 - f, f, 0.2, 1.0
            m.colors.append(c)
        arr = MarkerArray()
        arr.markers.append(m)
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


def main(args=None):
    rclpy.init(args=args)
    node = RacingDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
