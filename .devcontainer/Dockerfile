FROM osrf/ros:humble-desktop

SHELL ["/bin/bash", "-c"]

ENV RCUTILS_COLORIZED_OUTPUT=1

# Install required packages
RUN apt update && \
    apt upgrade -y && \
    apt install -y \
        bash-completion \
        build-essential \
        clang-format \
        cmake \
        gdb \
        git \
        htop \
        nano \
        sudo \
        wget \
        vim \
        python3-flake8 \
        python3-pip \
        python3-setuptools \
        ros-$ROS_DISTRO-rmw-fastrtps-cpp \
        ros-$ROS_DISTRO-rmw-cyclonedds-cpp \
        ros-$ROS_DISTRO-plotjuggler \
        ros-$ROS_DISTRO-plotjuggler-ros \
        ros-dev-tools && \
    python3 -m pip install -U \
        argcomplete \
        pre-commit

# Build libfranka
RUN apt update && \
    apt install -y \
        libpoco-dev \
        libeigen3-dev && \
    git clone -b 0.9.2 --recursive https://github.com/frankaemika/libfranka /tmp/libfranka && \
    mkdir -p /tmp/libfranka/build && \
    cd /tmp/libfranka/build && \
    cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF .. && \
    make -j${nproc} && \
    make install

# Create the user
RUN groupadd --gid 1000 geppetto && \
    useradd --uid 1000 --gid 1000 -m geppetto && \
    echo geppetto ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/geppetto && \
    chmod 0440 /etc/sudoers.d/geppetto

USER geppetto
WORKDIR /home/geppetto

ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp

RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /home/geppetto/.bashrc && \
    echo "source /home/geppetto/ros2_ws/install/setup.bash" >> /home/geppetto/.bashrc && \
    echo "source /usr/share/gazebo/setup.sh" >> /home/geppetto/.bashrc && \
    echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> /home/geppetto/.bashrc