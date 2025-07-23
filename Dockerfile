FROM ros:humble-ros-core

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
ENV ROS_VERSION=2
ENV ROS_PYTHON_VERSION=3

RUN apt-get update && apt-get install -y \
    curl \
    wget \
    git \
    nano \
    htop \
    tree \
    screen \
    unzip \
    software-properties-common \
    build-essential \
    cmake \
    make \
    gcc \
    g++ \
    gdb \
    valgrind \
    pkg-config \
    libtool \
    autoconf \
    automake \
    python3-pip \
    python3-dev \
    python3-setuptools \
    python3-wheel \
    python3-numpy \
    python3-scipy \
    python3-matplotlib \
    python3-opencv \
    net-tools \
    iputils-ping \
    openssh-client \
    lsb-release \
    gnupg2 \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-rosdep \
    python3-vcstool \
    python3-flake8-docstrings \
    python3-pytest-cov \
    python3-argcomplete \
    ros-dev-tools \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y \
    ros-humble-rclcpp \
    ros-humble-rclpy \
    ros-humble-std-msgs \
    ros-humble-sensor-msgs \
    ros-humble-geometry-msgs \
    ros-humble-nav-msgs \
    ros-humble-tf2 \
    ros-humble-tf2-ros \
    ros-humble-tf2-geometry-msgs \
    ros-humble-tf2-tools \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-urdf \
    ros-humble-xacro \
    ros-humble-rviz2 \
    ros-humble-rviz-common \
    ros-humble-rviz-default-plugins \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-nav2-util \
    ros-humble-nav2-msgs \
    ros-humble-nav2-core \
    ros-humble-nav2-common \
    ros-humble-nav2-costmap-2d \
    ros-humble-nav2-planner \
    ros-humble-nav2-controller \
    ros-humble-nav2-recoveries \
    ros-humble-nav2-bt-navigator \
    ros-humble-nav2-waypoint-follower \
    ros-humble-nav2-lifecycle-manager \
    ros-humble-nav2-map-server \
    ros-humble-nav2-amcl \
    ros-humble-nav2-smac-planner \
    ros-humble-nav2-theta-star-planner \
    ros-humble-nav2-dwb-controller \
    ros-humble-nav2-regulated-pure-pursuit-controller \
    ros-humble-nav2-rotation-shim-controller \
    ros-humble-nav2-simple-commander \
    ros-humble-nav2-smoother \
    ros-humble-nav2-behaviors \
    ros-humble-slam-toolbox \
    ros-humble-cartographer \
    ros-humble-cartographer-ros \
    ros-humble-rtabmap-ros \
    ros-humble-rtabmap-sync \
    ros-humble-rtabmap-slam \
    ros-humble-rtabmap-viz \
    ros-humble-rtabmap-launch \
    ros-humble-rtabmap-util \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-image-transport-plugins \
    ros-humble-compressed-image-transport \
    ros-humble-compressed-depth-image-transport \
    ros-humble-robot-localization \
    ros-humble-amcl \
    ros-humble-teleop-twist-keyboard \
    ros-humble-teleop-twist-joy \
    ros-humble-joy \
    ros-humble-joystick-drivers \
    ros-humble-topic-tools \
    ros-humble-diagnostic-updater \
    ros-humble-angles \
    ros-humble-behaviortree-cpp-v3 \
    ros-humble-demo-nodes-cpp \
    ros-humble-demo-nodes-py \
    ros-humble-example-interfaces \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y \
    libeigen3-dev \
    libceres-dev \
    libgoogle-glog-dev \
    libgflags-dev \
    libatlas-base-dev \
    libsuitesparse-dev \
    libomp-dev \
    libvtk9-dev \
    libpcl-dev \
    libopencv-dev \
    libqt5core5a \
    libqt5gui5 \
    libqt5widgets5 \
    qt5-qmake \
    qtbase5-dev \
    lcov \
    python3-zmq \
    libgraphicsmagick++1-dev \
    graphicsmagick-libmagick-dev-compat \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep init || echo "rosdep already initialized"

WORKDIR /workspace

RUN echo '#!/bin/bash\n\
set -e\n\
\n\
rosdep update\n\
\n\
source /opt/ros/humble/setup.bash\n\
\n\
if [ -f "/workspace/package.xml" ]; then\n\
    echo "Found package.xml, installing dependencies..."\n\
    rosdep install --from-paths /workspace --ignore-src -r -y --rosdistro $ROS_DISTRO || true\n\
fi\n\
\n\
if [ -d "/workspace/src" ]; then\n\
    echo "Found src directory, installing dependencies from workspace..."\n\
    rosdep install --from-paths /workspace/src --ignore-src -r -y --rosdistro $ROS_DISTRO || true\n\
    \n\
fi\n\
\n\
exec "$@"\n\
' > /entrypoint.sh && chmod +x /entrypoint.sh

RUN echo "# ROS2 Humble Setup" >> /root/.bashrc && \
    echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "export ROS_DOMAIN_ID=42" >> /root/.bashrc && \
    echo "alias source_ws='source install/setup.bash'" >> /root/.bashrc && \
    echo "alias cl='clear'" >> /root/.bashrc

ENTRYPOINT ["/entrypoint.sh"]

CMD ["bash"]