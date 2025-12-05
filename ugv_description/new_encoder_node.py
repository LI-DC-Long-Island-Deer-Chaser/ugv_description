#!/usr/bin/env python3
"""
Wheel Encoder Node (Refactored)

Responsibilities:
- Reads encoder data from Arduino via serial
- Publishes odometry at fixed rate (20 Hz)
- Runs PID velocity control loop at high frequency (100 Hz)
- Sends throttle commands via MAVROS RC override
- Subscribes to cmd_vel for target velocity

Architecture:
- EncoderReader: Parses serial data from Arduino
- OdometryCalculator: Converts encoder data to odometry
- PIDController: Velocity control with anti-windup
- Main Node: Orchestrates timers and data flow
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from mavros_msgs.msg import OverrideRCIn
from datetime import datetime
import serial
import json
import math
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from ugv_description.pid_controller import PIDController


class WheelEncoderNode(Node):
    def __init__(self):
        super().__init__('wheel_encoder_node')
        
        self._declare_parameters()
        self._init_state()
        self._init_serial()
        self._init_publishers()
        self._init_subscribers()
        self._init_timers()
        
        self.get_logger().info('Node initialized')
        self.get_logger().info(f'Odom: {self.odom_rate} Hz | PID: {self.pid_rate} Hz')
    
    def _declare_parameters(self):
        """Declare all ROS parameters."""
        # Serial
        self.declare_parameter('serial_port', '/dev/ttyACM2')
        self.declare_parameter('baud_rate', 115200)
        
        # Rates
        self.declare_parameter('odom_publish_rate', 20.0)
        self.declare_parameter('pid_update_rate', 100.0)
        
        # RC PWM limits
        self.declare_parameter('throttle_pwm_neutral', 1500)
        self.declare_parameter('throttle_pwm_min', 1100)
        self.declare_parameter('throttle_pwm_max', 1900)

        # Robot Properties
        self.declare_parameter('ticks_per_rev', 357.74)
        self.declare_parameter('wheel_circumference', 0.38713)
        
        # Get values
        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.odom_rate = self.get_parameter('odom_publish_rate').value
        self.pid_rate = self.get_parameter('pid_update_rate').value
        self.throttle_neutral = self.get_parameter('throttle_pwm_neutral').value
        self.throttle_min = self.get_parameter('throttle_pwm_min').value
        self.throttle_max = self.get_parameter('throttle_pwm_max').value
        self.ticks_per_rev = self.get_parameter('ticks_per_rev').value
        self.wheel_circumference = self.get_parameter('wheel_circumference').value
    
    def _init_state(self):
        """Initialize shared state variables."""
        self.latest_encoder_position = 0
        self.latest_encoder_speed = 0.0
        self.latest_timestamp_us = 0
        self.target_lin_velocity = 0.0
        self.target_ang_velocity = 0.0

        self.pid = PIDController(Kp=1, Ki=1.5, Kd=0.2, ticks_per_revolution=self.ticks_per_rev, wheel_circumference=self.wheel_circumference)
    
    def _init_serial(self):
        """Open serial connection to Arduino."""
        try:
            self.serial = serial.Serial(
                port=self.serial_port,
                baudrate=self.baud_rate,
                timeout=0.1
            )
            self.get_logger().info(f'Serial: {self.serial_port} @ {self.baud_rate}')
        except Exception as e:
            self.get_logger().error(f'Serial failed: {e}')
            raise
    
    def _init_publishers(self):
        """Create all publishers."""
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.odom_pub = self.create_publisher(Odometry, 'wheel_odom/odom', qos)
        self.rc_override_pub = self.create_publisher(OverrideRCIn, '/mavros/rc/override', qos)
    
    def _init_subscribers(self):
        """Create all subscribers."""
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            qos
        )
    
    def _init_timers(self):
        """
        Create timer callbacks.
        
        Priority order (handled by ROS2 executor):
        1. read_serial_callback (200 Hz) - Updates shared state
        2. publish_odometry_callback (20 Hz) - Critical for localization
        3. pid_update_callback (100 Hz) - Can tolerate occasional delays
        """
        self.serial_timer = self.create_timer(0.005, self.read_serial_callback)
        self.odom_timer = self.create_timer(1.0 / self.odom_rate, self.publish_odometry_callback)
        self.pid_timer = self.create_timer(1.0 / self.pid_rate, self.pid_update_callback)
    
    # ================================================================
    # CALLBACKS
    # ================================================================
    
    def read_serial_callback(self):
        """
        Read encoder data from Arduino and update shared state.
        
        Expected JSON format from Arduino:
        {
            "position": 12345,      // encoder ticks
            "speed": 123.45,        // ticks/second
            "timestamp": 1234567    // microseconds
        }
        """
        try:
            if self.serial.in_waiting > 0:
                line = self.serial.readline().decode('utf-8').strip()
                if not line:
                    return
                
                data = json.loads(line)
                
                self.latest_encoder_position = data['position']
                self.latest_encoder_speed = data['speed']

                now = datetime.now()
                unix_timestamp = now.timestamp()
                self.latest_timestamp_us = int(unix_timestamp * 1_000_000.0)
                
        except json.JSONDecodeError:
            pass
        except Exception as e:
            self.get_logger().warn(f'Serial error: {e}')
    
    def cmd_vel_callback(self, msg: Twist):
        """
        Receive target velocity from cmd_vel.
        """
        self.target_lin_velocity = msg.linear.x
        self.target_ang_velocity = msg.angular.z
        self.get_logger().info('CMD_VEL Read')
    
    def publish_odometry_callback(self):
        """
        Compute and publish odometry at fixed rate.
        
        Uses OdometryCalculator to convert encoder data to Odometry message.
        This is the critical path for localization (EKF fusion).
        """
        # TODO: Use OdometryCalculator.update()
        # TODO: Publish to 'wheel_odom'
        pass
    
    def pid_update_callback(self):
        """
        Run PID velocity control loop and send throttle command.
        
        Flow:
        1. PIDController.update(target_vel, encoder_pos, timestamp) -> pwm
        2. Publish RC override message (channel 1 and 3)
        """
        pwm = self.pid.update(target_vel=self.target_lin_velocity, pos=self.latest_encoder_position, t_us=self.latest_timestamp_us)
        self.send_rc_override(throttle_pwm=pwm, steering_pwm=1500)
    
    def send_rc_override(self, throttle_pwm, steering_pwm):
        """
        Send throttle command via MAVROS RC override.
        
        Channel mapping:
        - channels[0] (RC1): Steering (handled by other node)
        - channels[2] (RC3): Throttle (controlled here)
        """
        rc_msg = OverrideRCIn()
        rc_msg.channels = [0] * 18
        
        rc_msg.channels[0] = steering_pwm  # Steering (not controlled here)
        rc_msg.channels[2] = throttle_pwm  # Throttle
        
        self.rc_override_pub.publish(rc_msg)

    def publish_odometry(self, x, y, theta, vx, vy, vtheta):
        """
        Publish odometry message.

        Args:
            x: X position in meters
            y: Y position in meters
            theta: Orientation in radians
            vx: Linear velocity in x (m/s)
            vy: Linear velocity in y (m/s)
            vtheta: Angular velocity (rad/s)
        """
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_footprint'

        # Position
        odom_msg.pose.pose.position.x = x
        odom_msg.pose.pose.position.y = y
        odom_msg.pose.pose.position.z = 0.0

        # Orientation (quaternion from theta)
        odom_msg.pose.pose.orientation.x = 0.0
        odom_msg.pose.pose.orientation.y = 0.0
        odom_msg.pose.pose.orientation.z = math.sin(theta / 2.0)
        odom_msg.pose.pose.orientation.w = math.cos(theta / 2.0)

        # Velocity
        odom_msg.twist.twist.linear.x = vx
        odom_msg.twist.twist.linear.y = vy
        odom_msg.twist.twist.angular.z = vtheta

        # Pose covariance (diagonal elements)
        odom_msg.pose.covariance[0] = 0.1   # x (good)
        odom_msg.pose.covariance[7] = 0.1   # y (good)
        odom_msg.pose.covariance[14] = 1.0  # z (not measured - high uncertainty)
        odom_msg.pose.covariance[21] = 1.0  # roll (not measured - high uncertainty)
        odom_msg.pose.covariance[28] = 1.0  # pitch (not measured - high uncertainty)
        odom_msg.pose.covariance[35] = 0.5  # yaw/theta (moderate - no gyro)

        # Twist covariance (diagonal elements)
        odom_msg.twist.covariance[0] = 0.01  # vx (very good from encoders)
        odom_msg.twist.covariance[7] = 0.1   # vy (not directly measured)
        odom_msg.twist.covariance[14] = 0.1  # vz (not measured)
        odom_msg.twist.covariance[21] = 0.1  # vroll (not measured)
        odom_msg.twist.covariance[28] = 0.1  # vpitch (not measured)
        odom_msg.twist.covariance[35] = 0.5  # vtheta (moderate uncertainty)

        self.odom_pub.publish(odom_msg)
    
    # ================================================================
    # CLEANUP
    # ================================================================
    
    def destroy_node(self):
        """Close serial connection on shutdown."""
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
