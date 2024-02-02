FROM nvcr.io/nvidia/pytorch:20.11-py3
ENV DEBIAN_FRONTEND=noninteractive

# System general setup
RUN apt-get update && apt-get install -y \
    wget \
    nano \
    git \
    python-pip \
    kmod \
    build-essential \
    x11-apps \
    iputils-ping \
    psmisc \
 && rm -rf /var/lib/apt/lists/*

## Install system dependencies
RUN apt update && apt install -y \
    gnupg2 \
    lsb-release \
    software-properties-common \
    wget \
 && rm -rf /var/lib/apt/lists/*

# Add ROS repository and key
RUN echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list \
 && wget http://packages.ros.org/ros.key -O - | apt-key add - \
 && apt update

# Install ROS Melodic and basic packages
RUN apt install -y \
    ros-melodic-desktop-full \
    ros-melodic-catkin \
 && rm -rf /var/lib/apt/lists/*

# Create catkin workspace
RUN mkdir -p /catkin_ws/src
WORKDIR /catkin_ws
# Install Python packages
RUN python -m pip install --upgrade pip \
 && pip install wrapt --upgrade --ignore-installed \
 && pip install tensorflow tensorboard packaging squaternion numpy

# Install Python packages
RUN pip install \
    catkin_pkg \
    empy \
    rosdep \
    rosinstall \
    rosinstall_generator \
    wstool \
    vcstools \
    pyyaml \
    attrs \
    PySide2 \
    rospkg \
    PyQt5

# Initialize rosdep
RUN rosdep init && rosdep update
ENV ROS_MASTER_URI=http://localhost:11311
ENV ROS_PORT_SIM=11311
ENV GAZEBO_RESOURCE_PATH=/catkin_ws/src/TD3_navigation/TD3_simulator/multi_robot_scenario/launch

RUN cd /catkin_ws/src && \
    git clone https://github.com/srl-freiburg/pedsim_ros.git && \
    cd pedsim_ros && \
    git submodule update --init --recursive

# Install ROS dependencies using rosdep and build the workspace
RUN /bin/bash -c '. /opt/ros/melodic/setup.bash; cd /catkin_ws; catkin_make; . /catkin_ws/devel/setup.bash'
RUN echo ". /opt/ros/melodic/setup.bash" >> ~/.bashrc
RUN echo ". /catkin_ws/devel/setup.bash" >> ~/.bashrc
RUN echo "ALL Done"