#!/usr/bin/env python3
"""
Wheel Encoder Odometry Node
Reads quadrature encoder data from Arduino over serial and publishes odometry.
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import serial
import json
import math
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


class WheelEncoderNode(Node):
    def __init__(self):
        super().__init__('wheel_encoder_node')
        
        # Parameters
        self.declare_parameter('serial_port', '/dev/ttyACM2')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('counts_per_foot', 281.66)  # Empirical calibration
        self.declare_parameter('wheel_base', 0.254)  # 10 inches = 0.254m between front/rear axles
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        
        # Get parameters
        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.counts_per_foot = self.get_parameter('counts_per_foot').value
        self.wheel_base = self.get_parameter('wheel_base').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        
        # Convert counts/foot to counts/meter
        self.counts_per_meter = self.counts_per_foot / 0.3048
        
        # State variables
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_encoder_count = None
        self.last_time = None
        
        # Publishers
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.odom_pub = self.create_publisher(Odometry, 'wheel_odom', qos)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Serial connection
        try:
            self.serial = serial.Serial(
                port=self.serial_port,
                baudrate=self.baud_rate,
                timeout=0.1
            )
            self.get_logger().info(f'Connected to {self.serial_port} at {self.baud_rate} baud')
        except Exception as e:
            self.get_logger().error(f'Failed to open serial port: {e}')
            raise
        
        # Timer to read serial data (runs as fast as data arrives)
        self.create_timer(0.005, self.read_serial)  # 200 Hz (matches Arduino)
        
        self.get_logger().info('Wheel encoder node initialized')
        self.get_logger().info(f'Calibration: {self.counts_per_meter:.2f} counts/meter')
    
    def read_serial(self):
        """Read and process serial data from Arduino."""
        try:
            if self.serial.in_waiting > 0:
                line = self.serial.readline().decode('utf-8').strip()
                
                if not line:
                    return
                
                # Parse JSON
                data = json.loads(line)
                
                # Extract encoder data (Arduino sends "position", not "pos")
                position = data['position']  # Total encoder counts
                speed = data['speed']        # Counts/second from Arduino
                acceleration = data.get('acceleration', 0.0)  # Counts/second^2 (optional)
                
                # Convert to ROS time
                current_time = self.get_clock().now()
                
                # Initialize on first reading
                if self.last_encoder_count is None:
                    self.last_encoder_count = position
                    self.last_time = current_time
                    return
                
                # Calculate distance traveled
                delta_counts = position - self.last_encoder_count
                delta_distance = delta_counts / self.counts_per_meter  # meters
                
                # Calculate time delta
                dt = (current_time - self.last_time).nanoseconds / 1e9
                if dt <= 0:
                    return
                
                # Calculate velocity (m/s)
                linear_velocity = delta_distance / dt
                
                # Update position (assume straight line for now, steering handled by EKF)
                delta_x = delta_distance * math.cos(self.theta)
                delta_y = delta_distance * math.sin(self.theta)
                
                self.x += delta_x
                self.y += delta_y
                
                # Publish odometry
                self.publish_odometry(current_time, linear_velocity)
                
                # Update state
                self.last_encoder_count = position
                self.last_time = current_time
                
        except json.JSONDecodeError as e:
            # Ignore malformed JSON (partial reads happen at startup)
            # Only log if it's not just an empty line
            if len(line) > 0:
                self.get_logger().debug(f'JSON decode error: {e} | Line: {line[:50]}')
        except KeyError as e:
            self.get_logger().warn(f'Missing JSON key {e} in: {line[:50]}')
        except Exception as e:
            self.get_logger().warn(f'Serial read error: {e}')
    
    def publish_odometry(self, timestamp, linear_velocity):
        """Publish odometry message."""
        odom = Odometry()
        odom.header.stamp = timestamp.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        
        # Position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        
        # Orientation (quaternion from theta)
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)
        
        # Velocity (only linear.x, no angular - steering unknown)
        odom.twist.twist.linear.x = linear_velocity
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = 0.0
        
        # Covariance (wheel odometry is good for velocity, poor for position over time)
        # Position covariance (increases over time due to drift)
        odom.pose.covariance[0] = 0.1   # x
        odom.pose.covariance[7] = 0.1   # y
        odom.pose.covariance[14] = 1.0  # z (not used)
        odom.pose.covariance[21] = 1.0  # roll (not used)
        odom.pose.covariance[28] = 1.0  # pitch (not used)
        odom.pose.covariance[35] = 0.5  # yaw (poor without steering feedback)
        
        # Velocity covariance (encoder is accurate for velocity)
        odom.twist.covariance[0] = 0.01  # vx (good)
        odom.twist.covariance[7] = 0.1   # vy (not measured)
        odom.twist.covariance[14] = 0.1  # vz (not used)
        odom.twist.covariance[21] = 0.1  # vroll (not used)
        odom.twist.covariance[28] = 0.1  # vpitch (not used)
        odom.twist.covariance[35] = 0.5  # vyaw (unknown without steering)
        
        self.odom_pub.publish(odom)
        
        # Optionally publish TF (usually let EKF handle this)
        # Uncomment if you want wheel odom to publish its own TF
        # self.publish_tf(timestamp)
    
    def publish_tf(self, timestamp):
        """Publish TF transform."""
        t = TransformStamped()
        t.header.stamp = timestamp.to_msg()
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(self.theta / 2.0)
        t.transform.rotation.w = math.cos(self.theta / 2.0)
        
        self.tf_broadcaster.sendTransform(t)
    
    def destroy_node(self):
        """Clean up serial connection."""
        if hasattr(self, 'serial') and self.serial.is_open:
            self.serial.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WheelEncoderNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
