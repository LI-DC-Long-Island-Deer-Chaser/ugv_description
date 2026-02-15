#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu
import tf2_ros
from tf2_geometry_msgs import do_transform_vector3
from geometry_msgs.msg import Vector3Stamped

class IMUTransformer(Node):
    def __init__(self):
        super().__init__('imu_transformer')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        imu_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.subscription = self.create_subscription(Imu, 'ap/imu/experimental/data', self.imu_callback, imu_qos)
        self.publisher = self.create_publisher(Imu, 'ap/imu/corrected/data', imu_qos)

    def transform_vector(self, vector, transform):
        """Helper to transform a Vector3 using a looked-up transform."""
        v = Vector3Stamped()
        v.vector = vector
        transformed = do_transform_vector3(v, transform)
        return transformed.vector

    def imu_callback(self, msg):
        try:
            # Look up the transform from the IMU frame to base_link
            transform = self.tf_buffer.lookup_transform(
                'base_link', msg.header.frame_id, rclpy.time.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return

        corrected_msg = Imu()
        corrected_msg.header = msg.header
        corrected_msg.header.stamp = self.get_clock().now().to_msg()
        corrected_msg.header.frame_id = 'base_link'

        corrected_msg.angular_velocity = self.transform_vector(
            msg.angular_velocity, transform)
        corrected_msg.linear_acceleration = self.transform_vector(
            msg.linear_acceleration, transform)

        # Apply rotation to orientation quaternion
        q_rot = transform.transform.rotation
        q_orig = msg.orientation
        corrected_msg.orientation.x = q_rot.w * q_orig.x + q_rot.x * q_orig.w + q_rot.y * q_orig.z - q_rot.z * q_orig.y
        corrected_msg.orientation.y = q_rot.w * q_orig.y - q_rot.x * q_orig.z + q_rot.y * q_orig.w + q_rot.z * q_orig.x
        corrected_msg.orientation.z = q_rot.w * q_orig.z + q_rot.x * q_orig.y - q_rot.y * q_orig.x + q_rot.z * q_orig.w
        corrected_msg.orientation.w = q_rot.w * q_orig.w - q_rot.x * q_orig.x - q_rot.y * q_orig.y - q_rot.z * q_orig.z

        corrected_msg.orientation_covariance = msg.orientation_covariance
        corrected_msg.angular_velocity_covariance = msg.angular_velocity_covariance
        corrected_msg.linear_acceleration_covariance = msg.linear_acceleration_covariance

        self.publisher.publish(corrected_msg)

def main(args=None):
    rclpy.init(args=args)
    node = IMUTransformer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
