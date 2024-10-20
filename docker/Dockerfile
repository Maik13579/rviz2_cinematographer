ARG BASE_IMAGE=ros:humble
FROM ${BASE_IMAGE} AS base
ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

# Add entrypoint
COPY docker/ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]

# Install dependencies
RUN apt update \
    && apt install -y \
    ros-$ROS_DISTRO-rviz2 \
    ros-$ROS_DISTRO-rqt* \
    ros-$ROS_DISTRO-cv-bridge \
    ros-$ROS_DISTRO-pluginlib \
    libboost-all-dev \
    qtbase5-dev \
    qtdeclarative5-dev \
    qt5-qmake \
    libqt5core5a \
    libqt5widgets5 \
    libqt5gui5 \
    build-essential \
    libglu1-mesa-dev freeglut3-dev mesa-common-dev \
    && rm -rf /var/lib/apt/lists/*

# get package xmls first to use cache during build
ENV ROS2_WS=/root/ros2_ws
COPY rviz2_cinematographer_msgs/package.xml $ROS2_WS/src/rviz2_cinematographer_msgs/package.xml
COPY rviz2_cinematographer_view_controller/package.xml $ROS2_WS/src/rviz2_cinematographer_view_controller/package.xml
COPY rviz2_cinematographer_gui/package.xml $ROS2_WS/src/rviz2_cinematographer_gui/package.xml

# Install ros dependencies, assume rosdep is initialized in base image
RUN apt update \
    && apt install -y qtbase5-dev qt5-qmake qtbase5-dev-tools \
    && rosdep update \
    && rosdep install --from-paths $ROS2_WS/src --ignore-src -r -y \
    && rm -rf /var/lib/apt/lists/*

# Get package sources
ENV ROS2_WS=/root/ros2_ws
COPY rviz2_cinematographer_msgs $ROS2_WS/src/rviz2_cinematographer_msgs
COPY rviz2_cinematographer_view_controller $ROS2_WS/src/rviz2_cinematographer_view_controller
COPY rviz2_cinematographer_gui $ROS2_WS/src/rviz2_cinematographer_gui

FROM base AS build
WORKDIR $ROS2_WS
RUN source /opt/ros/$ROS_DISTRO/setup.bash \
    && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release \
    && echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /root/.bashrc \
    && echo "source $ROS2_WS/install/setup.bash" >> /root/.bashrc
