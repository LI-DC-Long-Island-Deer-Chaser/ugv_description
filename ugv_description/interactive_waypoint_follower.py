#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PointStamped, PoseStamped
from geographic_msgs.msg import GeoPoint
from robot_localization.srv import FromLL
from utils.gps_utils import latLonYaw2Geopose

class InteractiveGpsWpCommander(Node):
    """
    ROS2 node to send gps waypoints to nav2 received from mapviz's point click publisher
    """

    def __init__(self):
        super().__init__(node_name="gps_wp_commander")
        self.navigator = BasicNavigator("basic_navigator")
        self.fromLL_client = self.create_client(FromLL, '/fromLL')

        self.mapviz_wp_sub = self.create_subscription(
            PointStamped, "/clicked_point", self.mapviz_wp_cb, 1)

    def mapviz_wp_cb(self, msg: PointStamped):
        """
        clicked point callback, sends received point to nav2 gps waypoint follower if its a geographic point
        """
        if msg.header.frame_id != "wgs84":
            self.get_logger().warning(
                f"Received point from mapviz in '{msg.header.frame_id}' frame instead of 'wgs84'. "
                "Please configure Mapviz's Point Click Publisher to use 'wgs84' as its target frame.")
            return

        self.navigator.waitUntilNav2Active(navigator='bt_navigator', localizer='bt_navigator')
        
        while not self.fromLL_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('fromLL service not available, waiting...')

        # Request cartesian conversion
        req = FromLL.Request()
        req.ll_point = GeoPoint()
        req.ll_point.latitude = msg.point.y
        req.ll_point.longitude = msg.point.x
        req.ll_point.altitude = 0.0

        future = self.fromLL_client.call_async(req)
        # Add a callback to be executed when the service response is received
        future.add_done_callback(lambda fut: self.on_fromLL_response(fut, latLonYaw2Geopose(msg.point.y, msg.point.x)))

    def on_fromLL_response(self, future, geopose):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error(f'Service call failed {e}')
            return

        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position = response.map_point
        pose.pose.orientation = geopose.orientation

        wp = [pose]
        self.navigator.followWaypoints(wp)
        
        if (self.navigator.isTaskComplete()):
            self.get_logger().info("wps completed successfully")


def main():
    rclpy.init()
    gps_wpf = InteractiveGpsWpCommander()
    rclpy.spin(gps_wpf)


if __name__ == "__main__":
    main()