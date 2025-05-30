# Created by Nelson Durrant, Oct 2024
FROM ros:noetic-ros-base

# Define a username and password for the new user
ARG NAME=snelsondurrant
ARG PASS=snelsondurrant

# Update and upgrade
RUN apt update && apt upgrade -y

# Install dependencies needed below
RUN apt update && apt install -y \
    sudo \
    curl

# Set up a new user
RUN useradd -ms /bin/bash $NAME
RUN usermod -aG sudo $NAME
RUN usermod -aG dialout $NAME
RUN echo "$NAME:$PASS" | chpasswd
USER $NAME
WORKDIR /home/$NAME

# Set up locale
USER root
RUN apt update && sudo apt install locales
RUN locale-gen en_US en_US.UTF-8
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
RUN export LANG=en_US.UTF-8
USER $NAME

# Add ROS apt repository
USER root
RUN apt install -y software-properties-common
RUN add-apt-repository universe
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
USER $NAME

# Install ROS tools
USER root
RUN apt update && apt install -y \
    python3-flake8-docstrings \
    python3-pip \
    python3-pytest-cov \
    ros-dev-tools
USER $NAME

# Install Ubuntu 20.04 dependencies
RUN python3 -m pip install -U \
    flake8-blind-except \
    flake8-builtins \
    flake8-class-newline \
    flake8-comprehensions \
    flake8-deprecated \
    flake8-import-order \
    flake8-quotes \
    "pytest>=5.3" \
    pytest-repeat \
    pytest-rerunfailures

# Build ROS 2 from source
RUN mkdir -p ~/ros2_humble/src
WORKDIR /home/$NAME/ros2_humble
RUN vcs import --input https://raw.githubusercontent.com/ros2/ros2/humble/ros2.repos src

USER root
RUN apt update && apt upgrade -y
RUN rm -rf /etc/ros/rosdep/sources.list.d/20-default.list
RUN rosdep init
RUN rosdep update
RUN rosdep install --from-paths src --ignore-src -y --skip-keys "fastcdr rti-connext-dds-6.0.1 urdfdom_headers"
USER $NAME

RUN colcon build --symlink-install

# Add tmux for scripting purposes
USER root
RUN apt update && apt install -y tmux
USER $NAME

WORKDIR /home/$NAME 