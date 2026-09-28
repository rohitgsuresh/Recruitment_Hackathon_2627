#!/usr/bin/env python3
"""Fifth Gear - team driver, version 1: a disparity-extender driver.

A reactive driver: it uses only the latest LiDAR scan to decide what to do.
It does not need a map or a planned racing line, so it is a good first
driver. Later versions can follow a precomputed racing line for more speed.

Method: the "disparity extender" (Nathan Otterness, UNC F1TENTH team, 2019).
Credit this in SUBMISSION.md.

Keep `driver` as the executable name and `/drive` as the output topic.
"""

import math

import numpy as np
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray

MAX_STEER = 0.4189  # [rad] steering limit of the f1tenth_gym car


class Driver(Node):

    def __init__(self):
        super().__init__('driver')

        # Wiring (unchanged from the template)
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('odom_topic', '/ego_racecar/odom')
        self.declare_parameter('drive_topic', '/drive')
        self.declare_parameter('max_range', 10.0)          # [m] clip the scan here

        # Tuning knobs - these are the numbers to experiment with
        self.declare_parameter('max_speed', 6.0)           # [m/s] on straights
        self.declare_parameter('min_speed', 1.5)           # [m/s] in the tightest turns
        self.declare_parameter('brake_gain', 1.5)          # speed <= brake_gain * distance ahead
        self.declare_parameter('fov_deg', 100.0)           # only look +/- this far to the sides
        self.declare_parameter('disparity_threshold', 0.3) # [m] jump that counts as an edge
        self.declare_parameter('extend_width', 0.30)       # [m] half car width (0.155) + margin
        self.declare_parameter('side_clearance', 0.25)     # [m] don't turn into things this close
        self.declare_parameter('steer_gain', 1.0)          # scale on the steering angle

        p = lambda name: self.get_parameter(name).value
        self.max_range = p('max_range')
        self.max_speed = p('max_speed')
        self.min_speed = p('min_speed')
        self.brake_gain = p('brake_gain')
        self.fov = math.radians(p('fov_deg'))
        self.disparity_threshold = p('disparity_threshold')
        self.extend_width = p('extend_width')
        self.side_clearance = p('side_clearance')
        self.steer_gain = p('steer_gain')

        # Ground-truth pose from the simulator (not used by v1 yet)
        self.position = None
        self.yaw = 0.0
        self.speed = 0.0

        self.drive_pub = self.create_publisher(
            AckermannDriveStamped, p('drive_topic'), 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/driver/markers', 1)

        self.create_subscription(LaserScan, p('scan_topic'), self.scan_callback, 1)
        self.create_subscription(Odometry, p('odom_topic'), self.odom_callback, 10)

        self._marker_divisor = 0
        self.get_logger().info('Fifth Gear driver v1 (disparity extender) is up.')

    # ------------------------------------------------------------------
    def odom_callback(self, msg):
        self.position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        q = msg.pose.pose.orientation
        self.yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                              1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.speed = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)

    def scan_callback(self, scan):
        ranges, angles = self.preprocess(scan)
        steering, speed = self.plan(ranges, angles)
        self.publish(steering, speed)

        self._marker_divisor = (self._marker_divisor + 1) % 10
        if self._marker_divisor == 0:
            self.publish_marker(steering)

    def preprocess(self, scan):
        ranges = np.asarray(scan.ranges, dtype=np.float64)
        ranges = np.nan_to_num(ranges, nan=0.0, posinf=self.max_range, neginf=0.0)
        ranges = np.clip(ranges, 0.0, self.max_range)
        angles = scan.angle_min + np.arange(len(ranges)) * scan.angle_increment
        return ranges, angles

    # ==================================================================
    # The driving logic
    # ==================================================================
    def plan(self, ranges, angles):
        # 1. Only look ahead. Angle 0 is straight ahead, positive is left.
        ahead = np.abs(angles) <= self.fov
        r = ranges[ahead]
        a = angles[ahead]
        increment = a[1] - a[0]

        # 2. Disparity extension: at every sudden jump from near to far, the
        #    near obstacle is "widened" over the far side by extend_width.
        #    The car then never aims at a gap narrower than about
        #    2 * extend_width, which also seals the gaps between cones.
        extended = r.copy()
        jumps = np.nonzero(np.abs(np.diff(r)) > self.disparity_threshold)[0]
        for i in jumps:
            if r[i] < r[i + 1]:
                near = r[i]              # near beam at i, far side is i+1 upward
            else:
                near = r[i + 1]          # near beam at i+1, far side is i downward
            if near < 0.01:
                continue                 # invalid reading, skip it
            n = int(math.ceil(math.atan2(self.extend_width, near) / increment))
            if r[i] < r[i + 1]:
                span = slice(i + 1, min(i + 1 + n, len(extended)))
            else:
                span = slice(max(i - n + 1, 0), i + 1)
            extended[span] = np.minimum(extended[span], near)

        # 3. Aim at the farthest open point. If several are about equally far,
        #    prefer the one closest to straight ahead (smoother driving).
        candidates = np.nonzero(extended >= extended.max() - 0.1)[0]
        target = candidates[np.argmin(np.abs(a[candidates]))]
        steering = float(np.clip(a[target] * self.steer_gain, -MAX_STEER, MAX_STEER))

        # 4. Don't cut corners: if something is right beside the car on the
        #    side we want to turn towards, go straight until it is clear.
        left_side = (angles > math.radians(60)) & (angles < math.radians(110))
        right_side = (angles < -math.radians(60)) & (angles > -math.radians(110))
        if steering > 0 and np.min(ranges[left_side]) < self.side_clearance:
            steering = 0.0
        if steering < 0 and np.min(ranges[right_side]) < self.side_clearance:
            steering = 0.0

        # 5. Speed: fast when straight, slow in turns, and never faster than
        #    the distance to whatever is directly ahead allows.
        straightness = 1.0 - abs(steering) / MAX_STEER
        speed = self.min_speed + (self.max_speed - self.min_speed) * straightness
        front = np.min(ranges[np.abs(angles) < math.radians(5)])
        speed = min(speed, self.brake_gain * front)
        speed = max(speed, 0.5)          # never stop completely (15 s stuck = DNF)

        return steering, speed

    # ------------------------------------------------------------------
    def publish(self, steering, speed):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.steering_angle = float(steering)
        msg.drive.speed = float(speed)
        self.drive_pub.publish(msg)

    def publish_marker(self, target_angle):
        marker = Marker()
        marker.header.frame_id = 'ego_racecar/base_link'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'team_driver'
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.scale.x, marker.scale.y, marker.scale.z = 1.5, 0.15, 0.15
        marker.color.g, marker.color.b, marker.color.a = 0.8, 1.0, 0.9
        marker.pose.orientation.z = math.sin(target_angle / 2.0)
        marker.pose.orientation.w = math.cos(target_angle / 2.0)
        array = MarkerArray()
        array.markers.append(marker)
        self.marker_pub.publish(array)


def main(args=None):
    rclpy.init(args=args)
    node = Driver()
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
