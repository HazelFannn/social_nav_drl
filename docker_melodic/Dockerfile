FROM nvcr.io/nvidia/pytorch:19.07-py3
ENV DEBIAN_FRONTEND=noninteractive

# System general setup
RUN apt-get update && \
    apt-get install -y wget nano git python3-pip kmod build-essential x11-apps iputils-ping psmisc

## Install system dependencies
RUN apt update && \
    apt install -y gnupg2 lsb-release software-properties-common wget

# Add ROS repository and key
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && \
    apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# Install ROS Melodic and basic packages
RUN apt update && \
    apt install -y ros-melodic-desktop-full ros-melodic-catkin

# Install Python dependencies
RUN apt install -y python3-rosdep python3-empy

# Initialize rosdep
RUN rosdep init && rosdep update
# Create catkin workspace
RUN mkdir -p /catkin_ws/src
# Install ROS dependencies using rosdep and build the workspace
RUN /bin/bash -c '. /opt/ros/melodic/setup.bash; cd /catkin_ws; catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3; . /catkin_ws/devel/setup.bash'

# Install Python 3.7 (or newer version)
RUN apt-get install -y software-properties-common && \
    add-apt-repository ppa:deadsnakes/ppa && \
    apt-get update && \
    apt-get install -y python3.7 python3.7-dev python3.7-venv && \
    update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.7 1

# Install Python packages
RUN python3 -m pip install --upgrade pip && \
    pip install wrapt --upgrade --ignore-installed && \
    pip install torch tensorflow tensorboard packaging squaternion numpy

ENV ROS_MASTER_URI=http://localhost:11311
ENV ROS_PORT_SIM=11311
ENV GAZEBO_RESOURCE_PATH=/catkin_ws/src/TD3_navigation/TD3_simulator/multi_robot_scenario/launch

RUN cd catkin_ws/src && \
    git clone https://github.com/srl-freiburg/pedsim_ros.git && \
    cd pedsim_ros && \
    git submodule update --init --recursive
RUN pip3 install pyyaml rospkg attrs
# RUN cd /catkin_ws/src/social_nav_drl/DRL-robot-navigation/catkin_ws/src/multi_robot_scenario/xacro/p3dx/ && \
#     find . -iname "*.xacro" | xargs sed -i 's#<\([/]\?\)\(if\|unless\|include\|arg\|property\|macro\|insert_block\)#<\1xacro:\2#g'
RUN /bin/bash -c '. /opt/ros/melodic/setup.bash; cd /catkin_ws; catkin_make;. ./devel/setup.bash'
RUN echo ". /opt/ros/melodic/setup.bash" >> ~/.bashrc
RUN echo ". /catkin_ws/devel/setup.bash" >> ~/.bashrc

RUN echo "ALL Done"
