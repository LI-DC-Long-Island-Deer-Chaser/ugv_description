#!/usr/bin/env python3
"""
AP Odom Bridge — fuses SLAM pose + wheel encoder velocity → /ap/odom

Architecture:
  Cartographer TF (map→base_link)  ──┐
                                      ├──→ nav_msgs/Odometry → /ap/odom → ArduPilot
  robot_localization /odom (twist) ──┘

Pose comes from Cartographer's TF tree (map→base_link lookup), giving
SLAM-corrected, loop-closed position.

Velocity comes from robot_localization's /odom topic, which is a direct
wheel encoder measurement (low-noise, no differentiation).

The combined Odometry message is published at a fixed rate (default 50Hz)
with BEST_EFFORT QoS to match ArduPilot's DDS subscriber.

Test mode (no Cartographer / robot_localization required):
  ros2 run ugv_description ap_odom_bridge.py --ros-args -p test_mode:=true

Normal mode:
  ros2 run ugv_description ap_odom_bridge.py

Parameters:
  vel_topic    (str,   default '/odom')  Odometry topic for twist (robot_localization)
  tf_parent    (str,   default 'map')    TF parent frame (Cartographer SLAM frame)
  tf_child     (str,   default 'base_link')  TF child frame
  publish_rate (float, default 50.0)     Output rate in Hz
  test_mode    (bool,  default False)    Publish static identity odom (no deps)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from tf2_ros import Buffer, TransformListener, LookupException, ExtrapolationException
from geometry_msgs.msg import TransformStamped


class ApOdomBridge(Node):
    def __init__(self):
        super().__init__('ap_odom_bridge')

        # ── parameters ────────────────────────────────────────────────
        self.declare_parameter('vel_topic', '/odom')
        self.declare_parameter('tf_parent', 'map')
        self.declare_parameter('tf_child', 'base_link')
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('test_mode', False)

        vel_topic = self.get_parameter('vel_topic').value
        self.tf_parent = self.get_parameter('tf_parent').value
        self.tf_child = self.get_parameter('tf_child').value
        publish_rate = self.get_parameter('publish_rate').value
        self.test_mode = self.get_parameter('test_mode').value

        # ── state ─────────────────────────────────────────────────────
        self._ap_stamp = None
        self._latest_twist = None       # twist from robot_localization
        self._tf_available = False
        self.msg_count = 0

        # ── QoS matching ArduPilot DDS (BEST_EFFORT / VOLATILE) ──────
        ap_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        # ── publisher ─────────────────────────────────────────────────
        self.ap_pub = self.create_publisher(Odometry, '/ap/odom', ap_qos)

        # ── subscribe to AP clock ─────────────────────────────────────
        self.clock_sub = self.create_subscription(
            Clock, '/ap/clock', self._ap_clock_cb, ap_qos
        )

        if self.test_mode:
            # ── test mode: static identity odom ───────────────────────
            self.timer = self.create_timer(1.0 / publish_rate, self._publish_test)
            self.get_logger().info(
                f'AP Odom Bridge started in TEST MODE at {publish_rate} Hz'
            )
        else:
            # ── TF listener for Cartographer pose ─────────────────────
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)

            # ── velocity subscriber (robot_localization) ──────────────
            vel_qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST,
                depth=5,
            )
            self.vel_sub = self.create_subscription(
                Odometry, vel_topic, self._vel_cb, vel_qos
            )

            # ── timer to publish combined odom ────────────────────────
            self.timer = self.create_timer(1.0 / publish_rate, self._publish_fused)

            self.get_logger().info(
                f'AP Odom Bridge started:\n'
                f'  Pose:     TF {self.tf_parent} → {self.tf_child} (Cartographer)\n'
                f'  Velocity: {vel_topic} (robot_localization)\n'
                f'  Output:   /ap/odom at {publish_rate} Hz'
            )

    # ── callbacks ─────────────────────────────────────────────────────
    def _ap_clock_cb(self, msg: Clock):
        self._ap_stamp = msg.clock

    def _vel_cb(self, msg: Odometry):
        self._latest_twist = msg.twist

    def _get_stamp(self):
        if self._ap_stamp is not None:
            return self._ap_stamp
        return self.get_clock().now().to_msg()

    # ── main publish loop (normal mode) ───────────────────────────────
    def _publish_fused(self):
        """Look up SLAM pose from TF, merge with latest twist, publish."""
        stamp = self._get_stamp()

        # Look up Cartographer's SLAM-corrected pose
        try:
            tf: TransformStamped = self.tf_buffer.lookup_transform(
                self.tf_parent, self.tf_child, rclpy.time.Time()  # latest
            )
        except (LookupException, ExtrapolationException):
            if not self._tf_available:
                # Only log once to avoid spam
                if self.msg_count == 0:
                    self.get_logger().warn(
                        f'Waiting for TF: {self.tf_parent} → {self.tf_child}...',
                        throttle_duration_sec=5.0,
                    )
            return

        if not self._tf_available:
            self._tf_available = True
            self.get_logger().info(
                f'TF {self.tf_parent} → {self.tf_child} available — publishing'
            )

        # Build the combined Odometry message
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'

        # Pose from Cartographer TF (SLAM-corrected)
        t = tf.transform.translation
        r = tf.transform.rotation
        msg.pose.pose.position.x = t.x
        msg.pose.pose.position.y = t.y
        msg.pose.pose.position.z = t.z
        msg.pose.pose.orientation.x = r.x
        msg.pose.pose.orientation.y = r.y
        msg.pose.pose.orientation.z = r.z
        msg.pose.pose.orientation.w = r.w

        # Twist from robot_localization (wheel encoder velocity)
        if self._latest_twist is not None:
            msg.twist = self._latest_twist

        self.ap_pub.publish(msg)

        self.msg_count += 1
        if self.msg_count % 500 == 0:
            vel_status = 'OK' if self._latest_twist is not None else 'WAITING'
            vx = 0.0
            if self._latest_twist is not None:
                vx = self._latest_twist.twist.linear.x
            self.get_logger().info(
                f'[{self.msg_count}] '
                f'pos=({t.x:.2f}, {t.y:.2f}, {t.z:.2f})  '
                f'vel_x={vx:.2f} m/s  '
                f'twist={vel_status}'
            )

    # ── test mode ─────────────────────────────────────────────────────
    def _publish_test(self):
        msg = Odometry()
        msg.header.stamp = self._get_stamp()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'
        msg.pose.pose.orientation.w = 1.0
        self.ap_pub.publish(msg)

        self.msg_count += 1
        if self.msg_count % 500 == 0:
            self.get_logger().info(f'Published {self.msg_count} test odom msgs')


def main(args=None):
    rclpy.init(args=args)
    node = ApOdomBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
