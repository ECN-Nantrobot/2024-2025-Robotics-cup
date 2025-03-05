#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class ScanRelayNode(Node):
    def __init__(self):
        super().__init__('scan_relay')
        self.subscription = self.create_subscription(
            LaserScan,
            '/rplidar_plugin/out',  # Eingangs-Topic
            self.scan_callback,
            10
        )
        self.publisher = self.create_publisher(
            LaserScan,
            '/scan',  # Ausgangs-Topic
            10
        )
        self.get_logger().info('Relaying /rplidar_plugin/out -> /scan')

    def scan_callback(self, msg):
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ScanRelayNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
