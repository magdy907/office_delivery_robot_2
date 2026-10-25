import math
import rclpy
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

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.on_odom, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)

        self.start_x = None
        self.start_y = None
        self.current_x = None
        self.current_y = None
        self.front_clear = True
        self.completed = False

        self.timer = self.create_timer(0.05, self.on_timer)

    def on_odom(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        if self.start_x is None:
            self.start_x = x
            self.start_y = y
            self.get_logger().info(f'Starting from ({x:.2f}, {y:.2f})')
        self.current_x = x
        self.current_y = y

    def on_scan(self, scan: LaserScan):
        stop_dist = float(self.get_parameter('stop_clearance_m').value)
        clear = True
        for i, r in enumerate(scan.ranges):
            if not math.isfinite(r) or r < scan.range_min or r > scan.range_max:
                continue
            angle = math.atan2(
                math.sin(scan.angle_min + i * scan.angle_increment),
                math.cos(scan.angle_min + i * scan.angle_increment))
            if abs(angle) <= math.radians(20.0) and r < stop_dist:
                clear = False
                break
        self.front_clear = clear

    def on_timer(self):
        if self.completed or self.current_x is None:
            return

        target = float(self.get_parameter('distance').value)
        max_speed = float(self.get_parameter('speed').value)
        tolerance = float(self.get_parameter('tolerance_m').value)

        travelled = math.hypot(
            self.current_x - self.start_x,
            self.current_y - self.start_y)

        cmd = Twist()
        if not self.front_clear:
            self.get_logger().warning(
                'Corridor blocked! Waiting...', throttle_duration_sec=1.5)
            self.cmd_pub.publish(cmd)
            return

        speed = command_speed(travelled, target, max_speed, tolerance=tolerance)
        cmd.linear.x = speed
        self.cmd_pub.publish(cmd)

        if speed == 0.0:
            self.completed = True
            error = abs(target - travelled)
            self.get_logger().info(
                f'Complete! Target: {target:.3f} m | '
                f'Measured: {travelled:.3f} m | Error: {error:.4f} m')


def main(args=None):
    rclpy.init(args=args)
    node = OfficeDriveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
