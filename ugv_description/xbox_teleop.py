#!/usr/bin/env python3
"""Simple joystick to ArduPilot /ap/joy converter"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy


class JoyTeleop(Node):
    def __init__(self):
        super().__init__('joy_teleop')
        self.ap_joy_pub = self.create_publisher(Joy, '/ap/joy', 10)
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.get_logger().info('Joy teleop started: axis 1 -> /ap/joy[0] (throttle), axis 3 -> /ap/joy[2] (steering)')
    
    def joy_callback(self, joy_msg: Joy):
        ap_joy = Joy()
        ap_joy.header.stamp = self.get_clock().now().to_msg()
        
        # Create array with at least 4 axes, all initialized to NaN
        ap_joy.axes = [float('nan')] * 8  # ArduPilot supports up to 8 RC channels
        
        if len(joy_msg.axes) > 3:
            ap_joy.axes[0] = -joy_msg.axes[3]   # Left stick Y -> throttle (channel 0)
            ap_joy.axes[2] = joy_msg.axes[1]   # Right stick X -> steering (channel 2)
        
        self.ap_joy_pub.publish(ap_joy)


def main(args=None):
    rclpy.init(args=args)
    node = JoyTeleop()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()