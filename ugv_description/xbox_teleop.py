#!/usr/bin/env python3
"""Simple 1:1 joystick to cmd_vel converter"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist


class JoyTeleop(Node):
    def __init__(self):
        super().__init__('joy_teleop')
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.get_logger().info('Joy teleop started: axis 1 -> linear.x, axis 3 -> angular.z')
    
    def joy_callback(self, joy_msg: Joy):
        twist = Twist()
        if len(joy_msg.axes) > 3:
            twist.linear.x = joy_msg.axes[1]   # Left stick Y
            twist.angular.z = joy_msg.axes[3]  # Right stick X
        self.cmd_vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = JoyTeleop()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()