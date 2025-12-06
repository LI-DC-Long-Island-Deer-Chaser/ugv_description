#!/usr/bin/env python3
"""
cmd_vel to RC Override Controller
Converts Twist commands to MAVROS RC overrides with PID velocity control and Ackermann steering.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from mavros_msgs.msg import OverrideRCIn
from nav_msgs.msg import Odometry
import math


class CmdVelController(Node):
    def __init__(self):
        super().__init__('cmd_vel_controller')
        
        # Parameters - Vehicle Configuration
        self.declare_parameter('wheel_base', 0.254)  # 10 inches between front/rear axles
        self.declare_parameter('max_steering_angle', 0.785398)  # 45 degrees in radians
        
        # Parameters - RC PWM Limits
        self.declare_parameter('steering_pwm_min', 1100)  # Full left
        self.declare_parameter('steering_pwm_max', 1900)  # Full right
        self.declare_parameter('steering_pwm_center', 1500)
        
        # Traxxas XL-5 HV ESC throttle limits (3S)
        # ESC has 65 PWM deadband: 1455-1565 = no movement
        self.declare_parameter('throttle_pwm_min', 1390)  # Reverse limit (conservative)
        self.declare_parameter('throttle_pwm_max', 1610)  # Forward limit (conservative)
        self.declare_parameter('throttle_pwm_neutral', 1500)
        self.declare_parameter('throttle_pwm_forward_start', 1565)  # ESC forward deadband end
        self.declare_parameter('throttle_pwm_reverse_start', 1455)  # ESC reverse deadband end
        
        # Parameters - PID Gains (START WITH THESE, TUNE LATER)
        self.declare_parameter('kp', 100.0)  # Proportional gain
        self.declare_parameter('ki', 10.0)   # Integral gain
        self.declare_parameter('kd', 5.0)    # Derivative gain
        self.declare_parameter('max_integral', 50.0)  # Anti-windup limit
        
        # Get parameters
        self.wheel_base = self.get_parameter('wheel_base').value
        self.max_steering_angle = self.get_parameter('max_steering_angle').value
        self.steering_pwm_min = self.get_parameter('steering_pwm_min').value
        self.steering_pwm_max = self.get_parameter('steering_pwm_max').value
        self.steering_pwm_center = self.get_parameter('steering_pwm_center').value
        self.throttle_pwm_min = self.get_parameter('throttle_pwm_min').value
        self.throttle_pwm_max = self.get_parameter('throttle_pwm_max').value
        self.throttle_pwm_neutral = self.get_parameter('throttle_pwm_neutral').value
        self.throttle_pwm_forward_start = self.get_parameter('throttle_pwm_forward_start').value
        self.throttle_pwm_reverse_start = self.get_parameter('throttle_pwm_reverse_start').value
        
        # PID gains
        self.kp = self.get_parameter('kp').value
        self.ki = self.get_parameter('ki').value
        self.kd = self.get_parameter('kd').value
        self.max_integral = self.get_parameter('max_integral').value
        
        # PID state
        self.integral_error = 0.0
        self.last_error = 0.0
        self.last_time = None
        
        # Current velocity (from odometry feedback)
        self.current_velocity = 0.0
        
        # Desired velocity (from cmd_vel)
        self.desired_velocity = 0.0
        self.desired_angular = 0.0
        
        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        self.odom_sub = self.create_subscription(
            Odometry,
            'odometry/filtered',  # From EKF
            self.odom_callback,
            10
        )
        
        # Publisher
        self.rc_override_pub = self.create_publisher(
            OverrideRCIn,
            '/mavros/rc/override',
            10
        )
        
        # Control loop timer (50 Hz)
        self.create_timer(0.02, self.control_loop)
        
        self.get_logger().info('cmd_vel controller initialized')
        self.get_logger().info(f'ESC: Traxxas XL-5 HV (3S) with deadband compensation')
        self.get_logger().info(f'Throttle: Fwd [{self.throttle_pwm_forward_start}-{self.throttle_pwm_max}], '
                               f'Rev [{self.throttle_pwm_min}-{self.throttle_pwm_reverse_start}], '
                               f'Neutral={self.throttle_pwm_neutral}')
        self.get_logger().info(f'PID Gains - Kp: {self.kp}, Ki: {self.ki}, Kd: {self.kd}')
    
    def cmd_vel_callback(self, msg):
        """Receive desired velocity commands."""
        self.desired_velocity = msg.linear.x  # m/s
        self.desired_angular = msg.angular.z  # rad/s
    
    def odom_callback(self, msg):
        """Receive current velocity feedback from EKF."""
        self.current_velocity = msg.twist.twist.linear.x
    
    def control_loop(self):
        """Main control loop - runs at 50 Hz."""
        current_time = self.get_clock().now()
        
        # Initialize time on first run
        if self.last_time is None:
            self.last_time = current_time
            return
        
        # Calculate dt
        dt = (current_time - self.last_time).nanoseconds / 1e9
        if dt <= 0:
            return
        
        # ========== VELOCITY PID CONTROL ==========
        # Error
        error = self.desired_velocity - self.current_velocity
        
        # Proportional
        p_term = self.kp * error
        
        # Integral (with anti-windup)
        self.integral_error += error * dt
        self.integral_error = max(-self.max_integral, min(self.max_integral, self.integral_error))
        i_term = self.ki * self.integral_error
        
        # Derivative
        d_term = self.kd * (error - self.last_error) / dt
        
        # PID output
        pid_output = p_term + i_term + d_term
        
        # ========== ESC DEADBAND MAPPING ==========
        # Traxxas XL-5 HV ESC has 65 PWM deadband (1455-1565)
        # Map PID output around the deadband to active ranges
        
        # Velocity deadband - commands below this are treated as stop
        velocity_deadband = 0.05  # 5 cm/s
        
        if abs(self.desired_velocity) < velocity_deadband:
            # Stop command - send neutral
            throttle_pwm = self.throttle_pwm_neutral
            self.integral_error = 0.0  # Reset integral when stopped
        elif pid_output > 0:
            # Forward - map PID output to active range [1565, 1610]
            # Scale PID output to fit the active forward range
            active_range = self.throttle_pwm_max - self.throttle_pwm_forward_start
            throttle_pwm = self.throttle_pwm_forward_start + int(min(pid_output, active_range))
            # Clamp to max limit
            throttle_pwm = min(throttle_pwm, self.throttle_pwm_max)
        else:
            # Reverse - map PID output to active range [1455, 1390]
            # Scale negative PID output to fit the active reverse range
            active_range = self.throttle_pwm_reverse_start - self.throttle_pwm_min
            throttle_pwm = self.throttle_pwm_reverse_start - int(min(abs(pid_output), active_range))
            # Clamp to min limit
            throttle_pwm = max(throttle_pwm, self.throttle_pwm_min)
        
        # ========== ACKERMANN STEERING CONTROL ==========
        # Convert angular velocity to steering angle using Ackermann geometry
        # tan(steering_angle) = angular_velocity * wheelbase / linear_velocity
        if abs(self.desired_velocity) > 0.1:  # Only steer when moving
            steering_angle = math.atan2(self.desired_angular * self.wheel_base, self.desired_velocity)
        elif abs(self.desired_angular) > 0.01:  # Pure rotation (point turn)
            # For stationary turning, use maximum steering
            steering_angle = math.copysign(self.max_steering_angle, self.desired_angular)
        else:
            steering_angle = 0.0
        
        # Clamp steering angle
        steering_angle = max(-self.max_steering_angle, min(self.max_steering_angle, steering_angle))
        
        # Map steering angle to PWM
        # -45° (left) = 1100, 0° (center) = 1500, +45° (right) = 1900
        steering_ratio = steering_angle / self.max_steering_angle  # -1 to +1
        pwm_range = (self.steering_pwm_max - self.steering_pwm_min) / 2
        steering_pwm = int(self.steering_pwm_center + (steering_ratio * pwm_range))
        
        # ========== PUBLISH RC OVERRIDE ==========
        rc_msg = OverrideRCIn()
        rc_msg.channels = [0] * 18  # Initialize all channels
        
        # Channel mapping (typical for rovers):
        rc_msg.channels[0] = steering_pwm   # Channel 1: Steering
        rc_msg.channels[2] = throttle_pwm   # Channel 3: Throttle
        
        # Publish
        self.rc_override_pub.publish(rc_msg)
        
        # Update state for next iteration
        self.last_error = error
        self.last_time = current_time
        
        # Debug logging (throttle to 5 Hz)
        if int(current_time.nanoseconds / 1e9) % 0.2 < 0.02:
            self.get_logger().info(
                f'Vel: {self.current_velocity:.2f}/{self.desired_velocity:.2f} m/s | '
                f'Steering: {math.degrees(steering_angle):.1f}° | '
                f'PWM: T={throttle_pwm} S={steering_pwm} | '
                f'PID: P={p_term:.1f} I={i_term:.1f} D={d_term:.1f}',
                throttle_duration_sec=1.0
            )


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
