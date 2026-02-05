#!/usr/bin/env python3
"""
IMU Transformer Node
Transforms IMU data from base_link_ned (NED frame) to base_link frame.
The static transform between these frames is a 180° rotation about X-axis.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu


class ImuTransformer(Node):
    def __init__(self):
        super().__init__('imu_transformer')
        
        # QoS profile for ArduPilot (best_effort reliability)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscribe to ArduPilot IMU in base_link_ned frame
        self.subscription = self.create_subscription(
            Imu,
            '/ap/imu/experimental/data',
            self.imu_callback,
            qos_profile
        )
        
        # Publish transformed IMU in base_link frame (also best_effort to match Cartographer)
        self.publisher = self.create_publisher(
            Imu,
            '/imu/data',
            qos_profile
        )
        
        self.msg_count = 0
        self.get_logger().info('IMU Transformer started: base_link_ned -> base_link')
    
    def quaternion_multiply(self, q1, q2):
        """Multiply two quaternions: q1 * q2 (in x,y,z,w format)"""
        x1, y1, z1, w1 = q1
        x2, y2, z2, w2 = q2
        
        return [
            w1*x2 + x1*w2 + y1*z2 - z1*y2,
            w1*y2 - x1*z2 + y1*w2 + z1*x2,
            w1*z2 + x1*y2 - y1*x2 + z1*w2,
            w1*w2 - x1*x2 - y1*y2 - z1*z2
        ]
        
    def imu_callback(self, msg):
        """
        Transform IMU from NED to base_link frame.
        Static transform: base_link -> base_link_ned is 180° rotation about X.
        So inverse transform (base_link_ned -> base_link) is also 180° about X.
        """
        transformed_msg = Imu()
        # Use current ROS time to sync with laser scans (ArduPilot time may drift)
        transformed_msg.header.stamp = self.get_clock().now().to_msg()
        transformed_msg.header.frame_id = 'base_link'  # Change frame to base_link
        
        # 180° rotation about X-axis = quaternion [1, 0, 0, 0] in x,y,z,w format
        rot_x_180 = [1.0, 0.0, 0.0, 0.0]
        
        # Transform orientation quaternion
        q_ned = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        q_transformed = self.quaternion_multiply(rot_x_180, q_ned)
        
        transformed_msg.orientation.x = q_transformed[0]
        transformed_msg.orientation.y = q_transformed[1]
        transformed_msg.orientation.z = q_transformed[2]
        transformed_msg.orientation.w = q_transformed[3]
        transformed_msg.orientation_covariance = msg.orientation_covariance
        
        # Transform angular velocity (rotate vector by 180° about X)
        transformed_msg.angular_velocity.x = msg.angular_velocity.x
        transformed_msg.angular_velocity.y = -msg.angular_velocity.y
        transformed_msg.angular_velocity.z = -msg.angular_velocity.z
        transformed_msg.angular_velocity_covariance = msg.angular_velocity_covariance
        
        # Transform linear acceleration (rotate vector by 180° about X)
        transformed_msg.linear_acceleration.x = msg.linear_acceleration.x
        transformed_msg.linear_acceleration.y = -msg.linear_acceleration.y
        transformed_msg.linear_acceleration.z = -msg.linear_acceleration.z
        transformed_msg.linear_acceleration_covariance = msg.linear_acceleration_covariance
        
        self.publisher.publish(transformed_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ImuTransformer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
