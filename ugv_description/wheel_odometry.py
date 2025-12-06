#!/usr/bin/env python3
"""
Wheel Odometry and Steering Calculator

Handles:
- Odometry calculations (position integration from encoder data)
- Ackermann steering angle calculations
- No ROS dependencies - pure calculations
"""
import math


class WheelOdometry:
    def __init__(self, counts_per_rev, wheel_circumference):
        """
        Initialize odometry calculator
        
        Args:
            counts_per_rev: Encoder counts per wheel revolution
            wheel_circumference: Wheel circumference in meters
        """
        self.counts_per_rev = counts_per_rev
        self.wheel_circumference = wheel_circumference
        
        # Odometry state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        # Last encoder position for delta calculation
        self.last_position = None
    
    def update(self, current_position):
        """
        Update odometry based on new encoder position
        
        Args:
            current_position: Current encoder position (counts)
            
        Returns:
            tuple: (x, y, theta, delta_distance) or None if first call
        """
        # Initialize on first call
        if self.last_position is None:
            self.last_position = current_position
            return None
        
        # Calculate distance traveled
        delta_counts = current_position - self.last_position
        delta_distance = (delta_counts / self.counts_per_rev) * self.wheel_circumference
        
        # Update position (straight line assumption - EKF handles turns with IMU)
        delta_x = delta_distance * math.cos(self.theta)
        delta_y = delta_distance * math.sin(self.theta)
        self.x += delta_x
        self.y += delta_y
        
        # Update last position
        self.last_position = current_position
        
        return (self.x, self.y, self.theta, delta_distance)
    
    def reset(self):
        """Reset odometry to origin"""
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_position = None
    
    def set_pose(self, x, y, theta):
        """Set odometry pose manually"""
        self.x = x
        self.y = y
        self.theta = theta


class AckermannSteering:
    def __init__(self, wheel_base, max_steering_angle):
        """
        Initialize Ackermann steering calculator
        
        Args:
            wheel_base: Distance between front and rear axles (meters)
            max_steering_angle: Maximum steering angle in radians
        """
        self.wheel_base = wheel_base
        self.max_steering_angle = max_steering_angle
    
    def calculate_steering_angle(self, linear_vel, angular_vel):
        """
        Calculate steering angle from cmd_vel
        
        Args:
            linear_vel: Linear velocity (m/s)
            angular_vel: Angular velocity (rad/s)
            
        Returns:
            float: Steering angle in radians (clamped to max)
        """
        if abs(linear_vel) > 0.1:
            # Moving: calculate steering from angular velocity
            steering_angle = math.atan2(
                angular_vel * self.wheel_base,
                linear_vel
            )
        elif abs(angular_vel) > 0.01:
            # Point turn: max steering in direction of rotation
            steering_angle = math.copysign(self.max_steering_angle, angular_vel)
        else:
            # Stopped
            steering_angle = 0.0
        
        # Clamp to maximum steering angle
        steering_angle = max(-self.max_steering_angle, 
                            min(self.max_steering_angle, steering_angle))
        
        return steering_angle
    
    def angle_to_pwm(self, steering_angle, pwm_center, pwm_min, pwm_max):
        """
        Convert steering angle to PWM value
        
        Args:
            steering_angle: Steering angle in radians
            pwm_center: Center/neutral PWM value
            pwm_min: Minimum PWM value
            pwm_max: Maximum PWM value
            
        Returns:
            int: PWM value
        """
        # Calculate ratio of current angle to max angle
        steering_ratio = steering_angle / self.max_steering_angle
        
        # Convert to PWM
        pwm_range = (pwm_max - pwm_min) / 2
        pwm_value = int(pwm_center + (steering_ratio * pwm_range))
        
        # Clamp to limits
        pwm_value = max(pwm_min, min(pwm_max, pwm_value))
        
        return pwm_value
