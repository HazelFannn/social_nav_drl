FROM osrf/ros:melodic-desktop-full
ENV DEBIAN_FRONTEND noninteractive

# System general setup
RUN apt-get update && \
    apt-get install -y wget nano git python3-pip kmod build-essential x11-apps iputils-ping psmisc && \
    apt-get install -y ros-melodic-catkin ros-melodic-moveit ros-melodic-gazebo-ros-pkgs ros-melodic-gazebo-ros-control ros-melodic-ros-control ros-melodic-industrial-robot-status-controller && \
    mkdir -p /catkin_ws/src

# Install Python 3.7 (or newer version)
RUN apt-get install -y software-properties-common && \
    add-apt-repository ppa:deadsnakes/ppa && \
    apt-get update && \
    apt-get install -y python3.7 python3.7-dev python3.7-venv && \
    update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.7 1

# Install Python packages
RUN python3 -m pip install --upgrade pip && \
    pip3 install torch tensorflow tensorboard packaging squaternion numpy

# gazebo setup
RUN mkdir -p ~/.gazebo && \
    cd ~/.gazebo && \
    git clone https://github.com/osrf/gazebo_models.git && \
    mv gazebo_models models

# Copy the NVIDIA driver installer into the container
COPY NVIDIA-Linux-x86_64-535.129.03.run /tmp/NVIDIA-Linux-x86_64-535.129.03.run

# Install dependencies for driver installation and other necessary tools
RUN apt-get update && apt-get install -y \
    kmod \
    build-essential

# Make the installer executable and run it
RUN chmod +x /tmp/NVIDIA-Linux-x86_64-535.129.03.run && \
    /tmp/NVIDIA-Linux-x86_64-535.129.03.run -a -N --ui=none --no-kernel-module

# Clean up
RUN rm /tmp/NVIDIA-Linux-x86_64-535.129.03.run

RUN apt-get update && apt install -y nvidia-cuda-toolkit

ENV ROS_HOSTNAME=localhost
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
