#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
from geographic_msgs.msg import GeoPoint
from robot_localization.srv import FromLL
import yaml
from ament_index_python.packages import get_package_share_directory
import os
import sys
import time

from utils.gps_utils import latLonYaw2Geopose


class YamlWaypointParser:
    """
    Parse a set of gps waypoints from a yaml file
    """

    def __init__(self, wps_file_path: str) -> None:
        with open(wps_file_path, 'r') as wps_file:
            self.wps_dict = yaml.safe_load(wps_file)

    def get_wps(self):
        """
        Get an array of geographic_msgs/msg/GeoPose objects from the yaml file
        """
        gepose_wps = []
        for wp in self.wps_dict["waypoints"]:
            latitude, longitude, yaw = wp["latitude"], wp["longitude"], wp["yaw"]
            gepose_wps.append(latLonYaw2Geopose(latitude, longitude, yaw))
        return gepose_wps


class GpsWpCommander(Node):
    """
    Class to use nav2 gps waypoint follower to follow a set of waypoints logged in a yaml file
    """

    def __init__(self, wps_file_path):
        super().__init__(node_name="gps_wp_commander")
        self.navigator = BasicNavigator("basic_navigator")
        self.wp_parser = YamlWaypointParser(wps_file_path)
        self.fromLL_client = self.create_client(FromLL, '/fromLL')

    def start_wpf(self):
        """
        Function to start the waypoint following
        """
        self.navigator.waitUntilNav2Active(navigator='bt_navigator', localizer='bt_navigator')
        
        while not self.fromLL_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('fromLL service not available, waiting...')

        poses = []
        wps = self.wp_parser.wps_dict["waypoints"]

        for wp in wps:
            req = FromLL.Request()
            req.ll_point = GeoPoint()
            req.ll_point.latitude = wp["latitude"]
            req.ll_point.longitude = wp["longitude"]
            req.ll_point.altitude = 0.0

            future = self.fromLL_client.call_async(req)
            rclpy.spin_until_future_complete(self, future)

            try:
                response = future.result()
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.header.stamp = self.get_clock().now().to_msg()
                pose.pose.position = response.map_point
                
                geopose = latLonYaw2Geopose(wp["latitude"], wp["longitude"], wp["yaw"])
                pose.pose.orientation = geopose.orientation
                poses.append(pose)
            except Exception as e:
                self.get_logger().error(f'Service call failed {e}')
                return

        self.navigator.followWaypoints(poses)
        while (not self.navigator.isTaskComplete()):
            time.sleep(0.1)
        self.get_logger().info("wps completed successfully")


def main():
    rclpy.init()

    # allow to pass the waypoints file as an argument
    default_yaml_file_path = os.path.join(get_package_share_directory(
        "ugv_description"), "config", "irl", "gps", "irl_wps.yaml")
    if len(sys.argv) > 1:
        yaml_file_path = sys.argv[1]
    else:
        yaml_file_path = default_yaml_file_path

    gps_wpf = GpsWpCommander(yaml_file_path)
    gps_wpf.start_wpf()


if __name__ == "__main__":
    main()