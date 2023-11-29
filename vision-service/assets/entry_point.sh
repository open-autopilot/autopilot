# setting up standards
export ROS_DOMAIN_ID=16
export FASTRTPS_DEFAULT_PROFILES_FILE='client.xml'
source /opt/ros/humble/setup.bash

# resetting daemon
ros2 daemon stop
ros2 daemon start

#  starting nodes
ros2 run demo_nodes_cpp talker




