#!/usr/bin/env python3

import math
import time
import yaml

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


def yaw_to_quaternion_zw(yaw: float):
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def make_pose(navigator: BasicNavigator, x: float, y: float, yaw: float, frame_id: str) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.position.z = 0.0

    z, w = yaw_to_quaternion_zw(yaw)
    pose.pose.orientation.x = 0.0
    pose.pose.orientation.y = 0.0
    pose.pose.orientation.z = z
    pose.pose.orientation.w = w
    return pose


def load_patrol_file(path: str):
    with open(path, "r") as f:
        return yaml.safe_load(f) or {}


def safe_clear_costmaps(navigator: BasicNavigator):
    try:
        navigator.clearAllCostmaps()
    except Exception:
        pass


def main():
    rclpy.init()
    navigator = None

    try:
        patrol_file = "/home/odinroast/ugv_ws/src/ugv_description/config/irl/maps/patrol_points.yaml"
        config = load_patrol_file(patrol_file)

        frame_id = config.get("frame_id", "map")
        map_yaml = config.get("map_yaml", "")
        initial_pose_cfg = config.get("initial_pose")
        waypoints_cfg = config.get("waypoints", [])

        if not initial_pose_cfg:
            raise RuntimeError("No initial_pose found in patrol_points.yaml")

        if not waypoints_cfg:
            raise RuntimeError("No waypoints found in patrol_points.yaml")

        navigator = BasicNavigator()

        # Set initial pose
        init_pose = make_pose(
            navigator,
            initial_pose_cfg["x"],
            initial_pose_cfg["y"],
            initial_pose_cfg["yaw"],
            frame_id,
        )
        navigator.setInitialPose(init_pose)

        # Wait for Nav2/AMCL
        navigator.waitUntilNav2Active(localizer="amcl")

        if map_yaml:
            navigator.get_logger().info(f"Using map file reference: {map_yaml}")

        safe_clear_costmaps(navigator)

        # IMPORTANT:
        # This is ONE PASS through the waypoint list.
        # No lap loop.
        # No restarting.
        # No appending the first waypoint again.
        goal_poses = [
            make_pose(navigator, wp["x"], wp["y"], wp.get("yaw", 0.0), frame_id)
            for wp in waypoints_cfg
        ]

        navigator.get_logger().info(f"Submitting one-shot route with {len(goal_poses)} goals...")
        navigator.goThroughPoses(goal_poses)

        while rclpy.ok() and not navigator.isTaskComplete():
            time.sleep(0.25)

        if not rclpy.ok():
            navigator.get_logger().warn("ROS shutting down before route completed.")
            return

        result = navigator.getResult()

        if result == TaskResult.SUCCEEDED:
            navigator.get_logger().info("One-shot patrol route completed successfully.")
        elif result == TaskResult.FAILED:
            navigator.get_logger().warn("One-shot patrol route failed.")
        elif result == TaskResult.CANCELED:
            navigator.get_logger().warn("One-shot patrol route was canceled.")
        else:
            navigator.get_logger().warn(f"Unknown result: {result}")

    except KeyboardInterrupt:
        pass

    finally:
        if navigator is not None:
            try:
                navigator.destroyNode()
            except Exception:
                pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()