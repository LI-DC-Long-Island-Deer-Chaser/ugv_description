import launch
import launch.actions
import launch.substitutions
import launch_ros.actions


def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package="mapviz",
            executable="mapviz",
            name="mapviz",
        ),
        launch_ros.actions.Node(
            package="swri_transform_util",
            executable="initialize_origin.py",
            name="initialize_origin",
            parameters=[
                {"local_xy_frame": "map"},
                {"local_xy_origin": "sbu"},
                {"local_xy_origins": """[
                    {"name": "sbu",
                        "latitude": 40.91354,
                        "longitude": -73.12524,
                        "altitude": 52.5,
                        "heading": 0.0},
                    {"name": "back_40",
                        "latitude": 40.91350,
                        "longitude": -73.12520,
                        "altitude": 50.0,
                        "heading": 0.0}
                ]"""},
            ]
        ),
        launch_ros.actions.Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="sbu_transform",
            arguments=["0", "0", "0", "0", "0", "0", "map", "origin"]
        )
    ])
