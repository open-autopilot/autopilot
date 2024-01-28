/lib/systemd/systemd-udevd --daemon
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | sudo tee /etc/udev/rules.d/80-movidius.rules
udevadm control --reload-rules && udevadm trigger

# setting up standards
export ROS_DOMAIN_ID=16
# export FASTRTPS_DEFAULT_PROFILES_FILE='client.xml'
source /opt/ros/humble/setup.bash
source /vision_mapping/install/setup.bash

# resetting daemon
ros2 daemon stop
ros2 daemon start

#  starting nodes
#ros2 launch depthai_ros_driver camera.launch.py
ros2 launch vision_mapping grid_launch.py



