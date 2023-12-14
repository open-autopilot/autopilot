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

# installing ros2 dependencies
RUN apt-get update
RUN apt-get install ros-humble-navigation2 -y
RUN apt-get install ros-humble-nav2-* -y
RUN apt-get install ros-humble-dwb-* -y
RUN apt-get install libgeographic-dev -y
RUN apt-get install ros-humble-geographic-msgs -y

# compiling ros2 dependencies
RUN git clone https://github.com/cra-ros-pkg/robot_localization.git
WORKDIR /robot_localization
RUN source /opt/ros/humble/setup.bash && colcon build --symlink-install

# compiling ros2 code
ARG PROJECT_DIR
WORKDIR /
COPY /navigation-service/src/autopilot_bringup autopilot_bringup
COPY /navigation-service/src/autopilot_control autopilot_control

WORKDIR /autopilot_bringup
RUN source /opt/ros/humble/setup.bash && colcon build --symlink-install

WORKDIR /autopilot_control
RUN source /opt/ros/humble/setup.bash && colcon build --symlink-install

# running ros2 packages
WORKDIR /
COPY /buildtools/assets/client.xml client.xml
COPY /navigation-service/assets/entry_point.sh entry_point.sh