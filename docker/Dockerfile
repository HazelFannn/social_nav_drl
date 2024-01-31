FROM nvcr.io/nvidia/pytorch:21.10-py3
ENV DEBIAN_FRONTEND=noninteractive

# System general setup
RUN apt-get update && \
    apt-get install -y wget nano git python3-pip kmod build-essential x11-apps iputils-ping psmisc

## Install system dependencies
RUN apt update && \
    apt install -y gnupg2 lsb-release software-properties-common wget

# Add ROS repository and key
# RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && \
#     apt-key adv --keyserver 'keyserver.ubuntu.com' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
# Install curl
RUN apt-get update && apt-get install -y curl

# Add ROS repository and key
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && \
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
    apt-get update

# Install ROS-Noetic
RUN apt-get update && \
    apt-get install -y ros-noetic-desktop-full && \
    apt-get install -y python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential python3-empy

# Initialize rosdep
RUN rosdep init && \
    rosdep update && \
    echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

# # Install ROS-Melodic
#     ## Add ROS source
# RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && \
#     ## Add ROS Key
#     curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
#     ## Install ROS Melodic
#     apt-get update && \
#     apt-get install -y ros-melodic-desktop-full && \
#     ## Initialize rosdep
#     apt-get -y install python-rosdep && \
#     rosdep init && \
#     rosdep update && \
#     echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc && \
#     ## Install tools for constructing and managing ROS packages
#     apt-get install -y python-rosinstall python-rosinstall-generator python-wstool build-essential

# # Install Python dependencies
# RUN apt install -y python3-rosdep python3-empy


# Create catkin workspace
RUN mkdir -p /catkin_ws/src
# Install ROS dependencies using rosdep and build the workspace
RUN /bin/bash -c '. /opt/ros/noetic/setup.bash; cd /catkin_ws; catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3; . /catkin_ws/devel/setup.bash'

# # Install Python 3.8
# RUN add-apt-repository ppa:deadsnakes/ppa && \
#     apt-get update && \
#     apt-get install -y python3.8 python3.8-dev python3.8-venv && \
#     update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.8 10

# # Install Python packages
# RUN python3 -m pip install --upgrade pip && \
#     pip install wrapt --upgrade --ignore-installed && \
#     pip install torch tensorflow tensorboard packaging squaternion numpy

# # Copy the NVIDIA driver installer into the container
# COPY NVIDIA-Linux-x86_64-535.129.03.run /tmp/NVIDIA-Linux-x86_64-535.129.03.run

# # Install dependencies for driver installation and other necessary tools
# RUN apt-get update && apt-get install -y \
#     kmod \
#     build-essential

# # Make the installer executable and run it
# RUN chmod +x /tmp/NVIDIA-Linux-x86_64-535.129.03.run && \
#     /tmp/NVIDIA-Linux-x86_64-535.129.03.run -a -N --ui=none --no-kernel-module

# # Clean up
# RUN rm /tmp/NVIDIA-Linux-x86_64-535.129.03.run

# RUN apt-get update && apt install -y nvidia-cuda-toolkit

# RUN distribution=$(. /etc/os-release;echo $ID$VERSION_ID) \
#     && curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | apt-key add - \
#     && curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | tee /etc/apt/sources.list.d/nvidia-docker.list \
#     && apt-get update \
#     && apt-get install -y nvidia-docker2

ENV ROS_HOSTNAME=localhost
ENV ROS_MASTER_URI=http://localhost:11311
ENV ROS_PORT_SIM=11311
ENV GAZEBO_RESOURCE_PATH=/catkin_ws/src/DRL-robot-navigation/catkin_ws/src/multi_robot_scenario/launch

RUN cd /catkin_ws/src && \
    git clone https://github.com/srl-freiburg/pedsim_ros.git && \
    cd pedsim_ros && \
    git submodule update --init --recursive
RUN cd /catkin_ws/src && \
    git clone -b main https://github.com/reiniscimurs/DRL-robot-navigation.git
RUN pip3 install pyyaml rospkg attrs netifaces
WORKDIR /catkin_ws/
# RUN cd /catkin_ws/src/social_nav_drl/catkin_ws/src/multi_robot_scenario/xacro/p3dx/ && \
#     find . -iname "*.xacro" | xargs sed -i 's#<\([/]\?\)\(if\|unless\|include\|arg\|property\|macro\|insert_block\)#<\1xacro:\2#g'
# RUN /bin/bash -c '. /opt/ros/melodic/setup.bash; cd /catkin_ws; catkin_make;. ./devel/setup.bash'
RUN echo ". /opt/ros/noetic/setup.bash" >> ~/.bashrc
RUN echo ". /catkin_ws/devel/setup.bash" >> ~/.bashrc

RUN echo "ALL Done"