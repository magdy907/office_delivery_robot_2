"""Drive a measured distance, stopping for obstacles or missing sensor data.

AI-assisted update: startup interlock, receipt watchdog and scan validation.
The watchdog uses wall/steady time so it also runs when simulation time pauses.
This is a teaching controller, not a replacement for a simulator-side watchdog.
"""

import math
import time

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from office_delivery_robot.office_motion_rule import command_speed


class OfficeDriveNode(Node):
    def __init__(self):
        super().__init__('office_drive_node')
        self.declare_parameter('distance', 0.6)
        self.declare_parameter('speed', 0.15)
        self.declare_parameter('stop_clearance_m', 0.45)
        self.declare_parameter('tolerance_m', 0.02)
        self.declare_parameter('sensor_timeout_sec', 1.0)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.on_odom, qos_profile_sensor_data)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)

        self.start_x = None
        self.start_y = None
        self.current_x = None
        self.current_y = None
        self.front_clear = False
        self.scan_valid = False
        self.odom_valid = False
        self.front_distance = None
        self.last_odom_received = None
        self.last_scan_received = None
        self.completed = False
        self.last_warning = None
        self.last_warning_time = float('-inf')

        self.watchdog_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(
            0.05, self.on_timer, clock=self.watchdog_clock)

    def stop(self):
        self.cmd_pub.publish(Twist())

    def wait_stopped(self, message):
        self.stop()
        now = time.monotonic()
        if message != self.last_warning or now - self.last_warning_time >= 1.5:
            self.get_logger().warning(message)
            self.last_warning = message
            self.last_warning_time = now

    def on_odom(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.odom_valid = math.isfinite(x) and math.isfinite(y)
        if not self.odom_valid:
            return
        self.last_odom_received = time.monotonic()
        self.current_x = x
        self.current_y = y

    def on_scan(self, scan: LaserScan):
        self.last_scan_received = time.monotonic()
        self.scan_valid = False
        self.front_clear = False
        self.front_distance = None
        if not (math.isfinite(scan.angle_min)
                and math.isfinite(scan.angle_increment)
                and scan.angle_increment != 0.0
                and math.isfinite(scan.range_min)
                and math.isfinite(scan.range_max)
                and 0.0 <= scan.range_min < scan.range_max):
            return

        front_ranges = []
        for i, distance in enumerate(scan.ranges):
            angle = scan.angle_min + i * scan.angle_increment
            angle = math.atan2(math.sin(angle), math.cos(angle))
            if abs(angle) > math.radians(20.0):
                continue
            # Gazebo +inf means no return within the sensor's range.
            # NaN, -inf and out-of-range finite values are not clear-space proof.
            if distance == float('inf'):
                front_ranges.append(float(scan.range_max))
            elif math.isfinite(distance) and scan.range_min <= distance <= scan.range_max:
                front_ranges.append(float(distance))
            else:
                return
        if not front_ranges:
            return
        self.scan_valid = True
        self.front_distance = min(front_ranges)

    def on_timer(self):
        if self.completed:
            return

        target = float(self.get_parameter('distance').value)
        max_speed = float(self.get_parameter('speed').value)
        tolerance = float(self.get_parameter('tolerance_m').value)
        clearance = float(self.get_parameter('stop_clearance_m').value)
        timeout = float(self.get_parameter('sensor_timeout_sec').value)
        values = (target, max_speed, tolerance, clearance, timeout)
        if (not all(math.isfinite(v) for v in values)
                or target < 0.0 or max_speed <= 0.0 or tolerance < 0.0
                or clearance <= 0.0 or timeout <= 0.0):
            self.wait_stopped('Invalid parameters. Stopped.')
            return

        now = time.monotonic()
        if self.last_odom_received is None or self.last_scan_received is None:
            self.wait_stopped('Waiting for first /odom and /scan. Stopped.')
            return
        if now - self.last_odom_received > timeout:
            self.wait_stopped('Odometry timeout: /odom stopped arriving. Stopped.')
            return
        if now - self.last_scan_received > timeout:
            self.wait_stopped('Laser timeout: /scan stopped arriving. Stopped.')
            return
        if not self.odom_valid or not self.scan_valid:
            self.wait_stopped('Invalid odometry or front laser readings. Stopped.')
            return

        if self.start_x is None:
            self.start_x = self.current_x
            self.start_y = self.current_y
            self.get_logger().info(
                f'Starting from ({self.start_x:.2f}, {self.start_y:.2f})')
        travelled = math.hypot(
            self.current_x - self.start_x, self.current_y - self.start_y)
        if target - travelled <= tolerance:
            self.stop()
            self.completed = True
            self.get_logger().info(
                f'Complete! Target: {target:.3f} m | '
                f'Measured: {travelled:.3f} m | '
                f'Error: {abs(target - travelled):.4f} m')
            return

        self.front_clear = self.front_distance >= clearance
        if not self.front_clear:
            self.wait_stopped('Corridor blocked! Waiting...')
            return

        self.last_warning = None
        cmd = Twist()
        cmd.linear.x = command_speed(
            travelled, target, max_speed, tolerance=tolerance)
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = OfficeDriveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Best effort only: shutdown may already have invalidated the context.
        if rclpy.ok():
            node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
