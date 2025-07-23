#!/bin/bash

set -e

GITHUB_REPO="https://github.com/onurcalisir27/LearningAdaptive.git"

sudo apt update && sudo apt upgrade -y
sudo apt-get install -y \
    wget \
    git \
    htop \
    tree \
    screen \
    unzip \
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
    net-tools \
    iputils-ping \
    openssh-client \
    lsb-release \
    gnupg2 \

# verify environment
if ! locale | grep -q "UTF-8"; then
	sudo apt update
	sudo apt install -y locales
	sudo locale-gen en_US en_US.UTF-8
	sudo update-locale LC_ALL
	export LANG=en_US.UTF-8
fi

locale | grep -q "UTF-8" >> /dev/null

# installing ROS
sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y

export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')

curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb"

sudo dpkg -i /tmp/ros2-apt-source.deb

sudo apt update
sudo apt upgrade -y

sudo apt install ros-humble-desktop
sudo apt install -y ros-dev-tools
sudo apt install -y python3-rosdep
sudo apt install -y python3-colcon-common-extensions

source /opt/ros/humble/setup.bash

USERSHELL=$(basename "$SHELL")

case "$USERSHELL" in
  bash)
    echo "source /opt/ros/humble/setup.bash" >> /$HOME/.bashrc
    echo "export ROS_DOMAIN_ID=42" >> /$HOME/.bashrc
    ;;
  zsh)
    echo "source /opt/ros/humble/setup.zsh" >> /$HOME/.zshrc
    echo "export ROS_DOMAIN_ID=42" >> /$HOME/.zshrc
    ;;
  csh|tcsh)
    echo "#source /opt/ros/humble/setup.bash" >> /$HOME/.cshrc.nonlinear
    echo "#export ROS_DOMAIN_ID=42" >> /$HOME/.cshrc.nonlinear
    ;;
  *)
    echo "Couldn't find shell"
    ;;
esac

# installing pigpio
mkdir -p /$HOME/tmp
cd /$HOME/tmp
wget https://github.com/joan2937/pigpio/archive/master.zip
unzip master.zip
cd pigpio-master
make
sudo make install
cd /$HOME
rm -rf /$HOME/tmp

# creating the workspace
cd /$HOME
if [ -d "$HOME/LearningAdaptive" ]; then
  echo "Directory $HOME/LearningAdaptive already exists. Skipping git clone."
else
  git clone "$GITHUB_REPO"
fi

cd "$HOME/LearningAdaptive/src"
git pull origin

# install dependencies
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init
fi
rosdep update

cd /$HOME/LearningAdaptive
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble

# Make an initial build to make sure nothing went wrong
cd /$HOME/LearningAdaptive
source /opt/ros/humble/setup.bash
colcon build --symlink-install

# next steps
echo "ROS2 Humble was succesfully installed."
echo "Try opening a new tab, (CTRL+SHIFT+T), and run $"source install/setup.bash" to source your build."
echo "You are now ready to run ROS2 executables"
