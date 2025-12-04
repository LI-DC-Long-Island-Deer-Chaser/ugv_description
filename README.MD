Ruthvick's Version

ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true params_file:=/home/odinroast/Desktop/ros2_ws/src/ugv_description/config/nav2_params.yaml

hardware:
ros2 launch mavros apm.launch fcu_url:=/dev/ttyACM0:115200