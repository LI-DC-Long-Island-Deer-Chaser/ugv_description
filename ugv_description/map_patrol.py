#!/usr/bin/env python3

import math
import time
import yaml

import rclpy
from rclpy.time import Time
from rclpy.duration import Duration

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from lifecycle_msgs.srv import GetState
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from tf2_ros import Buffer, TransformListener


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
        return yaml.safe_load(f)


def wait_for_lifecycle_node_active(
    navigator: BasicNavigator,
    node_name: str,
    timeout_sec: float = 30.0,
):
    service_name = f"{node_name}/get_state"
    client = navigator.create_client(GetState, service_name)

    start = time.time()
    navigator.get_logger().info(f"Waiting for {service_name}...")

    while rclpy.ok() and not client.wait_for_service(timeout_sec=1.0):
        if time.time() - start > timeout_sec:
            raise RuntimeError(f"Timed out waiting for service {service_name}")

    req = GetState.Request()
    start = time.time()

    while rclpy.ok():
        future = client.call_async(req)
        rclpy.spin_until_future_complete(navigator, future, timeout_sec=1.0)

        if future.result() is not None:
            state = future.result().current_state.label
            navigator.get_logger().info(f"{node_name} state: {state}")
            if state == "active":
                return

        if time.time() - start > timeout_sec:
            raise RuntimeError(f"Timed out waiting for {node_name} to become active")

        time.sleep(0.2)


def wait_for_map_and_tf(
    navigator: BasicNavigator,
    map_topic: str = "/map",
    global_frame: str = "map",
    odom_frame: str = "odom",
    timeout_sec: float = 30.0,
):
    got_map = {"value": False}

    def map_cb(_msg: OccupancyGrid):
        got_map["value"] = True

    map_sub = navigator.create_subscription(OccupancyGrid, map_topic, map_cb, 10)

    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer, navigator, spin_thread=False)

    start = time.time()
    navigator.get_logger().info(f"Waiting for {map_topic} and {global_frame}->{odom_frame} TF...")

    while rclpy.ok():
        rclpy.spin_once(navigator, timeout_sec=0.1)

        have_tf = tf_buffer.can_transform(
            global_frame,
            odom_frame,
            Time(),
            timeout=Duration(seconds=0.0),
        )

        if got_map["value"] and have_tf:
            navigator.get_logger().info("Map topic and TF are ready.")
            navigator.destroy_subscription(map_sub)
            return tf_buffer, tf_listener

        if time.time() - start > timeout_sec:
            navigator.destroy_subscription(map_sub)
            raise RuntimeError(
                f"Timed out waiting for {map_topic} and {global_frame}->{odom_frame}"
            )

        time.sleep(0.1)


def main():
    rclpy.init()

    patrol_file = "/home/odinroast/ugv_ws/src/ugv_description/config/irl/maps/patrol_points.yaml"
    config = load_patrol_file(patrol_file)

    frame_id = config.get("frame_id", "map")
    waypoints_cfg = config.get("waypoints", [])

    if not waypoints_cfg:
        raise RuntimeError("No waypoints found in patrol_points.yaml")

    navigator = BasicNavigator()

    wait_for_lifecycle_node_active(navigator, "controller_server")
    wait_for_lifecycle_node_active(navigator, "planner_server")
    wait_for_lifecycle_node_active(navigator, "bt_navigator")
    wait_for_lifecycle_node_active(navigator, "behavior_server")

    wait_for_map_and_tf(
        navigator,
        map_topic="/map",
        global_frame="map",
        odom_frame="odom",
        timeout_sec=30.0,
    )

    navigator.clearAllCostmaps()

    lap = 0

    try:
        while rclpy.ok():
            lap += 1
            navigator.get_logger().info(f"Starting patrol lap {lap}")

            goal_poses = [
                make_pose(navigator, wp["x"], wp["y"], wp.get("yaw", 0.0), frame_id)
                for wp in waypoints_cfg
            ]

            # Start the task
            navigator.goThroughPoses(goal_poses)

            # HUMBLE-SAFE polling style: no task=...
            while not navigator.isTaskComplete():
                feedback = navigator.getFeedback()

                if feedback is not None:
                    try:
                        navigator.get_logger().info(
                            f"Distance remaining: {feedback.distance_remaining:.2f} m"
                        )
                    except Exception:
                        pass

                rclpy.spin_once(navigator, timeout_sec=0.1)
                time.sleep(0.1)

            result = navigator.getResult()

            if result == TaskResult.SUCCEEDED:
                navigator.get_logger().info(f"Lap {lap} complete. Restarting patrol...")
                time.sleep(1.0)
                continue

            elif result == TaskResult.FAILED:
                navigator.get_logger().warn(
                    "Patrol failed on this lap. Clearing costmaps and retrying..."
                )
                navigator.clearAllCostmaps()
                time.sleep(1.0)
                continue

            elif result == TaskResult.CANCELED:
                navigator.get_logger().warn(
                    "Patrol was canceled. Restarting in 1 second..."
                )
                time.sleep(1.0)
                continue

            else:
                navigator.get_logger().warn(
                    "Unknown result. Clearing costmaps and retrying..."
                )
                navigator.clearAllCostmaps()
                time.sleep(1.0)
                continue

    except KeyboardInterrupt:
        navigator.get_logger().info("Keyboard interrupt received. Stopping patrol.")

    finally:
        try:
            navigator.cancelTask()
        except Exception:
            pass
        navigator.destroyNode()
        rclpy.shutdown()


if __name__ == "__main__":
    main()