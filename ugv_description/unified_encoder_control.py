#!/usr/bin/env python3
"""
Unified Wheel Encoder & Control Node

Combines encoder reading, odometry publishing, and velocity control:
- Reads encoder data from Arduino at 200 Hz (direct serial)
- Publishes odometry to /wheel_odom at 20 Hz (for EKF fusion)
- Runs PID velocity control at 100 Hz (high-speed feedback)
- Sends RC override with ESC deadband compensation
- Publishes joint states for steering visualization

Uses modular OOP design:
- PIDController: Velocity control with buffer-based estimation
- WheelOdometry: Position integration from encoder
- AckermannSteering: Steering angle calculations
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from mavros_msgs.msg import OverrideRCIn
from sensor_msgs.msg import JointState
import serial
import json
import math
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from .pid_controller import PIDController
from .wheel_odometry import WheelOdometry, AckermannSteering


class UnifiedEncoderControlNode(Node):
    def __init__(self):
        super().__init__('wheel_encoder_control_node')
        
        self._declare_parameters()
        self._init_state()
        self._init_serial()
        self._init_publishers()
        self._init_subscribers()
        self._init_timers()
        
        self.get_logger().info('Unified Encoder & Control Node initialized')
        self.get_logger().info(f'Serial: {self.serial_port} @ {self.baud_rate} baud')
        self.get_logger().info(f'ESC: Traxxas XL-5 HV - PWM Range [{self.throttle_min}-{self.throttle_max}], Neutral {self.throttle_neutral}')
        self.get_logger().info(f'PID: Kp={self.kp}, Ki={self.ki}, Kd={self.kd} @ {self.pid_rate} Hz')
        self.get_logger().info(f'Odom: {self.odom_rate} Hz | Encoder: 357.74 counts/rev (weighted test)')
    
    def _declare_parameters(self):
        """Declare all ROS parameters."""
        # Serial communication
        self.declare_parameter('serial_port', '/dev/ttyACM2')
        self.declare_parameter('baud_rate', 115200)
        
        # Update rates
        self.declare_parameter('odom_publish_rate', 20.0)
        self.declare_parameter('pid_update_rate', 100.0)
        
        # Encoder calibration (weighted load test: 357.74 counts/rev)
        self.declare_parameter('counts_per_rev', 357.74)
        self.declare_parameter('wheel_circumference', 0.38713)  # meters (empirical)
        
        # Traxxas XL-5 HV ESC (3S) - calibrated PWM ranges (deadband handled in mapping)
        self.declare_parameter('throttle_pwm_neutral', 1500)
        self.declare_parameter('throttle_pwm_min', 1390)              # Full reverse
        self.declare_parameter('throttle_pwm_max', 1610)              # Full forward
        
        # Steering (True Ackermann geometry)
        self.declare_parameter('steering_pwm_min', 1100)
        self.declare_parameter('steering_pwm_max', 1900)
        self.declare_parameter('steering_pwm_center', 1500)
        self.declare_parameter('max_steering_angle', 0.7854)          # 45 degrees (0.7854 rad)
        self.declare_parameter('wheel_base', 0.336)                   # 336mm wheelbase
        self.declare_parameter('track_width', 0.212178)               # 212.178mm track width
        
        # PID gains (tuned for narrow 45 PWM range)
        self.declare_parameter('kp', 25.0)
        self.declare_parameter('ki', 0.08)
        self.declare_parameter('kd', 1.5)
        self.declare_parameter('max_integral', 10.0)
        
        # Velocity filtering (dual-stage)
        self.declare_parameter('velocity_filter_alpha', 0.3)
        
        # Frames
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        
        # Get all parameters
        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.odom_rate = self.get_parameter('odom_publish_rate').value
        self.pid_rate = self.get_parameter('pid_update_rate').value
        self.counts_per_rev = self.get_parameter('counts_per_rev').value
        self.wheel_circumference = self.get_parameter('wheel_circumference').value
        self.throttle_neutral = self.get_parameter('throttle_pwm_neutral').value
        self.throttle_min = self.get_parameter('throttle_pwm_min').value
        self.throttle_max = self.get_parameter('throttle_pwm_max').value
        self.steering_min = self.get_parameter('steering_pwm_min').value
        self.steering_max = self.get_parameter('steering_pwm_max').value
        self.steering_center = self.get_parameter('steering_pwm_center').value
        self.max_steering_angle = self.get_parameter('max_steering_angle').value
        self.wheel_base = self.get_parameter('wheel_base').value
        self.track_width = self.get_parameter('track_width').value
        self.kp = self.get_parameter('kp').value
        self.ki = self.get_parameter('ki').value
        self.kd = self.get_parameter('kd').value
        self.max_integral = self.get_parameter('max_integral').value
        self.filter_alpha = self.get_parameter('velocity_filter_alpha').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
    
    def _init_state(self):
        """Initialize all state variables and helper classes."""
        # Latest encoder data from serial (updated at 200 Hz)
        self.latest_position = 0
        self.latest_timestamp_us = 0
        
        # Command velocities from /cmd_vel
        self.target_lin_vel = 0.0
        self.target_ang_vel = 0.0
        
        # Current PWM outputs
        self.current_throttle_pwm = self.throttle_neutral
        self.current_steering_pwm = self.steering_center
        
        # Current steering angle (for odometry)
        self.current_steering_angle = 0.0
        
        # Initialize PID Controller
        self.pid_controller = PIDController(
            Kp=self.kp,
            Ki=self.ki,
            Kd=self.kd,
            ticks_per_revolution=self.counts_per_rev,
            wheel_circumference=self.wheel_circumference,
            pwm_neutral=self.throttle_neutral,
            pwm_min=self.throttle_min,
            pwm_max=self.throttle_max,
            buffer_size=20,
            filter_alpha=self.filter_alpha
        )
        
        # Initialize Wheel Odometry
        self.wheel_odom = WheelOdometry(
            counts_per_rev=self.counts_per_rev,
            wheel_circumference=self.wheel_circumference,
            wheel_base=self.wheel_base
        )
        
        # Initialize Ackermann Steering (True Ackermann with track width)
        self.ackermann_steering = AckermannSteering(
            wheel_base=self.wheel_base,
            track_width=self.track_width,
            max_steering_angle=self.max_steering_angle
        )
    
    def _init_serial(self):
        """Open serial connection to Arduino."""
        try:
            self.serial = serial.Serial(
                port=self.serial_port,
                baudrate=self.baud_rate,
                timeout=0.1
            )
            self.get_logger().info(f'Serial connected: {self.serial_port}')
        except Exception as e:
            self.get_logger().error(f'Serial connection failed: {e}')
            raise
    
    def _init_publishers(self):
        """Create all publishers."""
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.odom_pub = self.create_publisher(Odometry, 'wheel_odom', qos)
        self.rc_override_pub = self.create_publisher(OverrideRCIn, '/mavros/rc/override', qos)
        self.joint_state_pub = self.create_publisher(JointState, 'joint_states_encoder', qos)
    
    def _init_subscribers(self):
        """Create subscribers."""
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
    
    def _init_timers(self):
        """Create timer callbacks at different rates."""
        # High-speed serial reading (200 Hz - matches Arduino)
        self.create_timer(0.005, self.read_serial_callback)
        
        # Odometry publishing (20 Hz)
        self.create_timer(1.0 / self.odom_rate, self.publish_odometry_callback)
        
        # PID control loop (100 Hz)
        self.create_timer(1.0 / self.pid_rate, self.pid_control_callback)
        
        # Joint states (20 Hz - matches odometry)
        self.create_timer(1.0 / self.odom_rate, self.publish_joint_states_callback)
    
    # ========================================================================
    # CALLBACKS
    # ========================================================================
    
    def read_serial_callback(self):
        """
        Read encoder data from Arduino at 200 Hz.
        Expected JSON: {"position": 12345, "speed": 123.45, "timestamp": 1234567890}
        """
        try:
            if self.serial.in_waiting > 0:
                line = self.serial.readline().decode('utf-8').strip()
                if not line:
                    return
                
                data = json.loads(line)
                
                # Update latest encoder data
                self.latest_position = data['position']
                self.latest_timestamp_us = data.get('timestamp', 0)
                
        except json.JSONDecodeError:
            pass  # Ignore partial reads
        except KeyError as e:
            self.get_logger().warn(f'Missing key in JSON: {e}')
        except Exception as e:
            self.get_logger().warn(f'Serial error: {e}')
    
    def cmd_vel_callback(self, msg: Twist):
        """Receive target velocities from /cmd_vel."""
        self.target_lin_vel = msg.linear.x
        self.target_ang_vel = msg.angular.z
    
    def publish_odometry_callback(self):
        """
        Publish odometry at 20 Hz to /wheel_odom for EKF fusion.
        Uses WheelOdometry class for position integration with Ackermann steering.
        """
        current_time = self.get_clock().now()
        current_time_sec = current_time.nanoseconds / 1e9
        
        # Update odometry using WheelOdometry class with steering angle
        odom_result = self.wheel_odom.update(
            current_position=self.latest_position,
            steering_angle=self.current_steering_angle,
            current_time=current_time_sec
        )
        
        # Skip if first call (initialization)
        if odom_result is None:
            return
        
        x, y, theta, delta_distance, linear_vel, angular_vel = odom_result
        
        # Create and publish odometry message
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        
        # Position
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0
        
        # Orientation (quaternion from theta)
        odom.pose.pose.orientation.z = math.sin(theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(theta / 2.0)
        
        # Velocity (linear from odometry, angular from Ackermann steering)
        odom.twist.twist.linear.x = linear_vel
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = angular_vel
        
        # Covariance
        odom.pose.covariance[0] = 0.1    # x
        odom.pose.covariance[7] = 0.1    # y
        odom.pose.covariance[35] = 0.2   # yaw (better now with steering integration)
        odom.twist.covariance[0] = 0.01  # vx (accurate from encoders)
        odom.twist.covariance[35] = 0.1  # vyaw (from Ackermann calculation)
        
        self.odom_pub.publish(odom)
    
    def pid_control_callback(self):
        """
        PID velocity control at 100 Hz using PIDController class.
        Calculates steering using AckermannSteering class.
        Sends throttle + steering commands via RC override.
        """
        # ========== VELOCITY PID CONTROL ==========
        # PID controller handles velocity estimation, filtering, and PWM calculation
        throttle_pwm = self.pid_controller.update(
            target_vel=self.target_lin_vel,
            pos=self.latest_position,
            t_us=self.latest_timestamp_us
        )
        
        # Apply PWM directly (deadband calibrated in min/max range)
        if abs(self.target_lin_vel) < 0.05:
            # Stop command
            self.current_throttle_pwm = self.throttle_neutral
        else:
            # Map PID output to calibrated range [min, max]
            # PID controller automatically handles forward/reverse
            self.current_throttle_pwm = int(max(self.throttle_min, min(self.throttle_max, throttle_pwm)))
        
        # ========== ACKERMANN STEERING ==========
        # Calculate True Ackermann steering angles (center, inner, outer)
        center_angle, inner_angle, outer_angle = self.ackermann_steering.calculate_steering_angles(
            linear_vel=self.target_lin_vel,
            angular_vel=self.target_ang_vel
        )
        
        # Use center angle for single steering servo control
        # (Physical servo controls both wheels via mechanical linkage)
        # Store for odometry calculations
        self.current_steering_angle = center_angle
        
        # Convert center steering angle to PWM (1100-1900 μs)
        self.current_steering_pwm = self.ackermann_steering.angle_to_pwm(
            steering_angle=center_angle,
            pwm_center=self.steering_center,
            pwm_min=self.steering_min,
            pwm_max=self.steering_max
        )
        
        # ========== PUBLISH RC OVERRIDE ==========
        rc_msg = OverrideRCIn()
        rc_msg.channels = [0] * 18
        rc_msg.channels[0] = self.current_steering_pwm   # Channel 1: Steering
        rc_msg.channels[2] = self.current_throttle_pwm   # Channel 3: Throttle
        rc_msg.channels[3] = 2000                        # Channel 4: Low Gear

        self.get_logger().info(f'Target lin velocity: {self.target_lin_vel}')
        self.get_logger().info(f'Steering PWM: {self.current_steering_pwm}')
        self.get_logger().info(f'Throttle PWM: {self.current_throttle_pwm}')
        self.get_logger().info(f'Gear: {rc_msg.channels[3]}')
        self.rc_override_pub.publish(rc_msg)
    
    def publish_joint_states_callback(self):
        """
        Publish joint states for visualization (steering and wheels).
        Uses AckermannSteering class to convert PWM back to angle.
        """
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        
        # Calculate steering angle from PWM (reverse of angle_to_pwm)
        pwm_range = (self.steering_max - self.steering_min) / 2
        steering_ratio = (self.current_steering_pwm - self.steering_center) / pwm_range
        steering_angle = steering_ratio * self.max_steering_angle
        
        # Wheel rotation (estimate from velocity and time)
        # For now, just set based on throttle direction
        wheel_direction = 1.0 if self.current_throttle_pwm > self.throttle_neutral else -1.0
        
        # Joint names (match your URDF)
        joint_state.name = [
            'front_left_steering_joint',
            'front_right_steering_joint',
            'front_left_wheel_joint',
            'front_right_wheel_joint',
            'rear_left_wheel_joint',
            'rear_right_wheel_joint'
        ]
        
        # Joint positions
        joint_state.position = [
            steering_angle,    # Front left steering
            steering_angle,    # Front right steering
            0.0,              # Front left wheel (TODO: integrate based on velocity)
            0.0,              # Front right wheel
            0.0,              # Rear left wheel
            0.0               # Rear right wheel
        ]
        
        self.joint_state_pub.publish(joint_state)
    
    def destroy_node(self):
        """Clean up serial connection."""
        if hasattr(self, 'serial') and self.serial.is_open:
            self.serial.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UnifiedEncoderControlNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
