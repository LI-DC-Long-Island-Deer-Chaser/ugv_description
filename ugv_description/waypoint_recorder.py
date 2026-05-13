#!/usr/bin/env python3
"""
ros2 run ugv_description waypoint_recorder \
  --ros-args \
  -p output_file:=/home/odinroast/ugv_ws/src/ugv_description/config/irl/maps/patrol_points.yaml \
  -p map_yaml:=/home/odinroast/ugv_ws/src/ugv_description/config/irl/maps/my_m.yaml
"""
import math
import os
import yaml

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped


def quat_to_yaw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class PatrolPoseRecorder(Node):
    def __init__(self):
        super().__init__('patrol_pose_recorder')

        self.declare_parameter('output_file', 'patrol_points.yaml')
        self.declare_parameter('map_yaml', '')
        self.declare_parameter('frame_id', 'map')

        # Make these parameters so you can match whatever your RViz tools use.
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('initialpose_topic', '/initialpose')

        self.output_file = self.get_parameter('output_file').get_parameter_value().string_value
        self.map_yaml = self.get_parameter('map_yaml').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.goal_topic = self.get_parameter('goal_topic').get_parameter_value().string_value
        self.initialpose_topic = self.get_parameter('initialpose_topic').get_parameter_value().string_value

        self.data = {
            'map_yaml': self.map_yaml,
            'frame_id': self.frame_id,
            'initial_pose': None,
            'waypoints': []
        }

        if os.path.exists(self.output_file):
            try:
                with open(self.output_file, 'r') as f:
                    loaded = yaml.safe_load(f) or {}
                self.data.update(loaded)
                self.get_logger().info(f'Loaded existing file: {self.output_file}')
            except Exception as e:
                self.get_logger().warn(f'Could not load existing YAML: {e}')

        self.create_subscription(
            PoseWithCovarianceStamped,
            self.initialpose_topic,
            self.initial_pose_cb,
            10
        )

        self.create_subscription(
            PoseStamped,
            self.goal_topic,
            self.goal_pose_cb,
            10
        )

        self.get_logger().info('Patrol recorder ready.')
        self.get_logger().info(f'Initial pose topic: {self.initialpose_topic}')
        self.get_logger().info(f'Goal pose topic:    {self.goal_topic}')
        self.get_logger().info(f'Output file:        {self.output_file}')

    def save_yaml(self):
        with open(self.output_file, 'w') as f:
            yaml.safe_dump(self.data, f, sort_keys=False)

    def initial_pose_cb(self, msg: PoseWithCovarianceStamped):
        q = msg.pose.pose.orientation
        yaw = quat_to_yaw(q.x, q.y, q.z, q.w)

        self.data['initial_pose'] = {
            'x': float(msg.pose.pose.position.x),
            'y': float(msg.pose.pose.position.y),
            'yaw': float(yaw)
        }
        self.save_yaml()
        self.get_logger().info('Saved initial pose.')

    def goal_pose_cb(self, msg: PoseStamped):
        q = msg.pose.orientation
        yaw = quat_to_yaw(q.x, q.y, q.z, q.w)

        self.data.setdefault('waypoints', []).append({
            'x': float(msg.pose.position.x),
            'y': float(msg.pose.position.y),
            'yaw': float(yaw)
        })

        self.save_yaml()
        self.get_logger().info(
            f"Saved waypoint #{len(self.data['waypoints'])}: "
            f"x={msg.pose.position.x:.2f}, y={msg.pose.position.y:.2f}, yaw={yaw:.2f}"
        )


def main():
    rclpy.init()
    node = PatrolPoseRecorder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()