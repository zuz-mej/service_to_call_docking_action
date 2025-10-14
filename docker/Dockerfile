ARG ROS_DISTRO=jazzy
ARG BUILD_TEST=OFF

FROM husarnet/ros:${ROS_DISTRO}-ros-core

WORKDIR /ros2_ws

COPY . /ros2_ws/src/send_to_dock

RUN apt-get update --fix-missing && \
    apt upgrade -y && \
    apt-get install -y \
        ros-dev-tools && \
    # Install dependencies
    rm -rf /etc/ros/rosdep/sources.list.d/20-default.list && \
    rosdep init && \
    rosdep update --rosdistro $ROS_DISTRO && \
    rosdep install --from-paths src -y -i && \
    # Build
    source /opt/ros/$ROS_DISTRO/setup.bash && \
    colcon build --symlink-install --packages-select send_to_dock --cmake-args -DBUILD_TESTING=$BUILD_TEST && \
    # Get version
    echo $(cat src/send_to_dock/package.xml | grep '<version>' | sed -r 's/.*<version>([0-9]+.[0-9]+.[0-9]+)<\/version>/\1/g') >> /version.txt && \
    # Size optimization
    export SUDO_FORCE_REMOVE=yes && \
    apt-get remove -y \
        ros-dev-tools && \
    apt-get autoremove -y && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*
