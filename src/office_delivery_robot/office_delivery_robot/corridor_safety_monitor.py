import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan


class CorridorSafetyMonitor(Node):
    def __init__(self):
        super().__init__('corridor_safety_monitor')
        self.declare_parameter('safety_threshold_m', 1.0)
        self.declare_parameter('front_sector_deg', 25.0)
        self.clearance_pub = self.create_publisher(
            Float32, '/office_bot/front_clearance', 10)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)

    def on_scan(self, scan: LaserScan):
        half_sector = math.radians(
            float(self.get_parameter('front_sector_deg').value))
        min_front_dist = float('inf')

        for i, r in enumerate(scan.ranges):
            if not math.isfinite(r) or r < scan.range_min or r > scan.range_max:
                continue
            angle = scan.angle_min + i * scan.angle_increment
            angle = math.atan2(math.sin(angle), math.cos(angle))
            if abs(angle) <= half_sector and r < min_front_dist:
                min_front_dist = r

        if not math.isfinite(min_front_dist):
            return

        msg = Float32()
        msg.data = float(min_front_dist)
        self.clearance_pub.publish(msg)

        threshold = float(self.get_parameter('safety_threshold_m').value)
        if min_front_dist < threshold:
            self.get_logger().warning(
                f'Obstacle in corridor at {min_front_dist:.2f} m (< {threshold:.2f} m)')
        else:
            self.get_logger().info(
                f'Corridor clear: {min_front_dist:.2f} m', throttle_duration_sec=2.0)


def main(args=None):
    rclpy.init(args=args)
    node = CorridorSafetyMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
