import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
import os
from ament_index_python.packages import get_package_share_directory

gps_wpf_dir = get_package_share_directory("ugv_description")
mapviz_config_file = os.path.join(gps_wpf_dir, "config", "irl", "mapvizcfg.mvc")

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package="mapviz",
            executable="mapviz",
            name="mapviz",
            parameters=[{"config": mapviz_config_file}]
        ),
        launch_ros.actions.Node(
            package="swri_transform_util",
            executable="initialize_origin.py",
            name="initialize_origin",
           #parameters=[
           #    {"local_xy_frame": "map"},
           #    {"local_xy_origin": "auto"},
           #    {"local_xy_origins": """[
           #        {"name": "sbu",
           #            "latitude": 40.91354,
           #            "longitude": -73.12524,
           #            "altitude": 52.5,
           #            "heading": 0.0},
           #        {"name": "back_40",
           #            "latitude": 40.91350,
           #            "longitude": -73.12520,
           #            "altitude": 50.0,
           #            "heading": 0.0}
           #    ]"""},
           #]
            remappings=[
                ("fix", "ap/navsat"),
            ],
        ),
        launch_ros.actions.Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="swri_transform",
            arguments=["0", "0", "0", "0", "0", "0", "map", "origin"]
        )
    ])
