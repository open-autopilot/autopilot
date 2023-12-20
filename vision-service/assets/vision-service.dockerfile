FROM ubuntu:22.04
ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

# change the locale from POSIX to UTF-8
RUN apt-get update && apt-get install -y locales
RUN locale-gen en_US en_US.UTF-8 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8

# adding common and universe repository
RUN apt-get install -y software-properties-common && add-apt-repository universe

# adding ros repostories
RUN apt-get update && apt-get install curl -y
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null
RUN apt-get update && apt-get upgrade -y

# installing ros2 humble
RUN apt-get install ros-humble-ros-base -y
RUN apt-get install ros-dev-tools -y
RUN source /opt/ros/humble/setup.bash

# Install udev and ROS2 dependencies
RUN sudo apt-get update && \
    sudo apt-get install -y udev && \
    sudo apt-get install -y ros-humble-depthai-ros

# Add udev rule for Movidius
RUN echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | tee /etc/udev/rules.d/80-movidius.rules

# Start udev and trigger rules
RUN service udev start && \
sleep 2 && \  # Add a delay to ensure udev is fully initialized
udevadm control --reload-rules && \
udevadm trigger


ARG PROJECT_DIR
WORKDIR /
COPY /vision-service/src/vision_mapping vision_mapping
WORKDIR /vision_mapping
RUN source /opt/ros/humble/setup.bash && colcon build --symlink-install

# running ros2 packages
ARG PROJECT_DIR
WORKDIR /
COPY /buildtools/assets/client.xml client.xml
COPY /navigation-service/assets/entry_point.sh entry_point.sh