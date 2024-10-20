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
    ros-$ROS_DISTRO-rqt-common-plugins \
    && rm -rf /var/lib/apt/lists/*


ENV ROS2_WS=/root/ros2_ws
# TODO copy ros2 stuff here 

# Install ros dependencies, assume rosdep is initialized in base image
# RUN apt update \
#     && rosdep update \ 
#     && rosdep install --from-paths $ROS2_WS/src --ignore-src -r -y \
#     && rm -rf /var/lib/apt/lists/*

FROM base AS build
RUN source /opt/ros/$ROS_DISTRO/setup.bash \
    && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release \
    && echo "source $ROS2_WS/install/setup.bash" >> /root/.bashrc