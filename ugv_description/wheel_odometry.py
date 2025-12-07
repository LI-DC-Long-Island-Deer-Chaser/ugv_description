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
    def __init__(self, counts_per_rev, wheel_circumference, wheel_base):
        """
        Initialize odometry calculator
        
        Args:
            counts_per_rev: Encoder counts per wheel revolution
            wheel_circumference: Wheel circumference in meters
            wheel_base: Distance between front and rear axles (meters)
        """
        self.counts_per_rev = counts_per_rev
        self.wheel_circumference = wheel_circumference
        self.wheel_base = wheel_base
        
        # Odometry state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        # Last encoder position for delta calculation
        self.last_position = None
        self.last_time = None
    
    def update(self, current_position, steering_angle, current_time):
        """
        Update odometry based on new encoder position and steering angle
        
        Args:
            current_position: Current encoder position (counts)
            steering_angle: Current steering angle in radians
            current_time: Current timestamp (seconds)
            
        Returns:
            tuple: (x, y, theta, delta_distance, linear_vel, angular_vel) or None if first call
        """
        # Initialize on first call
        if self.last_position is None or self.last_time is None:
            self.last_position = current_position
            self.last_time = current_time
            return None
        
        # Calculate time delta
        dt = current_time - self.last_time
        if dt <= 0:
            return None
        
        # Calculate distance traveled
        delta_counts = current_position - self.last_position
        delta_distance = (delta_counts / self.counts_per_rev) * self.wheel_circumference
        
        # Calculate linear velocity
        linear_vel = delta_distance / dt
        
        # Calculate angular velocity using Ackermann steering relationship
        # angular_vel = (v * tan(steering_angle)) / wheel_base
        if abs(linear_vel) > 0.01:  # Only calculate if moving
            angular_vel = (linear_vel * math.tan(steering_angle)) / self.wheel_base
        else:
            angular_vel = 0.0
        
        # Update heading (theta) based on angular velocity
        delta_theta = angular_vel * dt
        self.theta += delta_theta
        
        # Normalize theta to [-pi, pi]
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        
        # Update position using current heading
        delta_x = delta_distance * math.cos(self.theta)
        delta_y = delta_distance * math.sin(self.theta)
        self.x += delta_x
        self.y += delta_y
        
        # Update last states
        self.last_position = current_position
        self.last_time = current_time
        
        return (self.x, self.y, self.theta, delta_distance, linear_vel, angular_vel)
    
    def reset(self):
        """Reset odometry to origin"""
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_position = None
        self.last_time = None
    
    def set_pose(self, x, y, theta):
        """Set odometry pose manually"""
        self.x = x
        self.y = y
        self.theta = theta


class AckermannSteering:
    def __init__(self, wheel_base, track_width, max_steering_angle):
        """
        Initialize True Ackermann steering calculator
        
        Args:
            wheel_base: Distance between front and rear axles (meters) - 0.336m
            track_width: Distance between left and right wheels (meters) - 0.212178m
            max_steering_angle: Maximum steering angle in radians (±45° = 0.7854 rad)
        """
        self.wheel_base = wheel_base
        self.track_width = track_width
        self.max_steering_angle = max_steering_angle
    
    def calculate_steering_angles(self, linear_vel, angular_vel):
        """
        Calculate true Ackermann steering angles from cmd_vel
        
        Uses proper Ackermann geometry:
        - Inner wheel: tighter angle (smaller turning radius)
        - Outer wheel: wider angle (larger turning radius)
        - Both wheels track around same instantaneous center
        
        Args:
            linear_vel: Linear velocity (m/s)
            angular_vel: Angular velocity (rad/s)
            
        Returns:
            tuple: (center_angle, inner_angle, outer_angle) all in radians, clamped to limits
        """
        if abs(linear_vel) < 0.01 and abs(angular_vel) > 0.01:
            # Point turn: max steering in direction of rotation
            center_angle = math.copysign(self.max_steering_angle, angular_vel)
            inner_angle = center_angle
            outer_angle = center_angle
        elif abs(angular_vel) < 0.001:
            # Straight line: no steering
            center_angle = 0.0
            inner_angle = 0.0
            outer_angle = 0.0
        else:
            # Calculate turning radius from cmd_vel
            turning_radius = linear_vel / angular_vel
            
            # Center steering angle (bicycle model)
            center_angle = math.atan(self.wheel_base / turning_radius)
            
            # True Ackermann: inner and outer wheel angles
            # Inner wheel has smaller radius (tighter turn)
            # Outer wheel has larger radius (wider turn)
            if turning_radius > 0:  # Left turn
                inner_angle = math.atan(self.wheel_base / (turning_radius - self.track_width / 2.0))
                outer_angle = math.atan(self.wheel_base / (turning_radius + self.track_width / 2.0))
            else:  # Right turn
                inner_angle = math.atan(self.wheel_base / (turning_radius + self.track_width / 2.0))
                outer_angle = math.atan(self.wheel_base / (turning_radius - self.track_width / 2.0))
        
        # Clamp all angles to maximum steering limit
        center_angle = max(-self.max_steering_angle, min(self.max_steering_angle, center_angle))
        inner_angle = max(-self.max_steering_angle, min(self.max_steering_angle, inner_angle))
        outer_angle = max(-self.max_steering_angle, min(self.max_steering_angle, outer_angle))
        
        return (center_angle, inner_angle, outer_angle)
    
    def angle_to_pwm(self, steering_angle, pwm_center, pwm_min, pwm_max):
        """
        Convert steering angle to PWM value (1100-1900 μs)
        
        Linear mapping from angle to PWM:
        - 0° (straight) → 1500 μs (center)
        - +max_angle (left) → 1900 μs (max)
        - -max_angle (right) → 1100 μs (min)
        
        Args:
            steering_angle: Steering angle in radians
            pwm_center: Center/neutral PWM value (1500)
            pwm_min: Minimum PWM value (1100)
            pwm_max: Maximum PWM value (1900)
            
        Returns:
            int: PWM value in microseconds
        """
        # Calculate ratio of current angle to max angle (-1.0 to +1.0)
        steering_ratio = steering_angle / self.max_steering_angle
        
        # Convert to PWM (linear mapping)
        pwm_range = (pwm_max - pwm_min) / 2.0
        pwm_value = int(pwm_center + (steering_ratio * pwm_range))
        
        # Clamp to limits
        pwm_value = max(pwm_min, min(pwm_max, pwm_value))
        
        return pwm_value
