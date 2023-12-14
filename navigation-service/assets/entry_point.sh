export ROS_DOMAIN_ID=16
export FASTRTPS_DEFAULT_PROFILES_FILE='client.xml'
source /opt/ros/humble/setup.bash
source /autopilot_bringup/install/setup.bash
source /robot_localization/install/setup.bash

ros2 daemon stop
ros2 daemon start

ros2 launch autopilot_bringup autopilot_bringup.launch.py

