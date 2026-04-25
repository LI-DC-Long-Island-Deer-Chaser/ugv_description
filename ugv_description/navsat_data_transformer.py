#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import NavSatFix

class GPSFrameFixer(Node):
    def __init__(self):
        super().__init__('navsat_data_transformer')

        gps_in_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        gps_out_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.subscription = self.create_subscription(NavSatFix, 'ap/navsat', self.gps_callback, gps_in_qos)
        self.publisher = self.create_publisher(NavSatFix, 'ap/navsat/corrected', gps_out_qos)

    def gps_callback(self, msg):
        corrected_msg = NavSatFix()
        corrected_msg.header = msg.header
        corrected_msg.header.frame_id = 'base_link'

        corrected_msg.status = msg.status
        corrected_msg.latitude = msg.latitude
        corrected_msg.longitude = msg.longitude
        corrected_msg.altitude = msg.altitude
        corrected_msg.position_covariance = msg.position_covariance
        corrected_msg.position_covariance_type = msg.position_covariance_type

        self.publisher.publish(corrected_msg)

def main(args=None):
    rclpy.init(args=args)
    node = GPSFrameFixer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()