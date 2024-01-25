FROM ros:humble-ros-core

SHELL ["/bin/bash", "-c"]

WORKDIR /ros2_ws


RUN apt update && \
    apt install -y \
        build-essential \
        cmake \
        git \
        libpoco-dev \
        libeigen3-dev && \
    git clone -b 0.9.2 --recursive https://github.com/frankaemika/libfranka /libfranka

RUN mkdir -p /libfranka/build && \
    cd /libfranka/build && \
    cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF .. && \
    make -j${nproc} && \
    make install

RUN apt update  && \
    apt install -y \
        python3-dev \
        python3-pip \
        python3-colcon-common-extensions && \
    pip3 install \
        rosdep \
        vcstool && \
    git clone https://github.com/tenfoldpaper/panda_ros2.git /ros2_ws/src/panda_ros2 && \
    cd /ros2_ws && \
    rm -rf src/panda_ros2/franka_moveit_config && \
    rosdep init && \
    rosdep update --rosdistro $ROS_DISTRO && \
    rosdep install --from-paths src -y -i  --skip-keys libfranka && \
    source /opt/ros/$ROS_DISTRO/setup.bash && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -Wall -Wextra -Wpedantic && \
    apt autoremove -y && \
    apt clean && \
    rm -rf /var/lib/apt/lists/*

COPY ./ros_entrypoint.sh / 
ENTRYPOINT ["/ros_entrypoint.sh"]
