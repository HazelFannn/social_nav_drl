FROM nvcr.io/nvidia/pytorch:21.10-py3
ENV DEBIAN_FRONTEND=noninteractive

# System general setup
RUN apt-get update && \
    apt-get install -y wget nano git python3-pip kmod build-essential x11-apps iputils-ping psmisc

# Update software list and install system layer
RUN apt-get update && \
    apt-get install -y lsb-release gnupg2 curl iputils-ping

# Install ROS-noetic
    ## Add ROS source
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && \
    ## Add ROS Key
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
    ## Install ROS noetic
    apt-get update && \
    apt-get install -y ros-noetic-desktop-full && \
    ## Initialize rosdep
    apt-get -y install python3-rosdep && \
    rosdep init && \
    rosdep update && \
    echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc && \
    ## Install tools for constructing and managing ROS packages
    apt-get install -y python3-rosinstall python3-rosinstall-generator python3-wstool build-essential

# Create catkin workspace
RUN mkdir -p /catkin_ws/src
# Install ROS dependencies using rosdep and build the workspace
RUN /bin/bash -c '. /opt/ros/noetic/setup.bash; cd /catkin_ws; catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3; . /catkin_ws/devel/setup.bash'
RUN python3 -m pip install --upgrade pip && \
    pip install wrapt --upgrade --ignore-installed && \
    pip install torch tensorflow tensorboard packaging squaternion numpy

ENV ROS_HOSTNAME=localhost
ENV ROS_MASTER_URI=http://localhost:11311
ENV ROS_PORT_SIM=11311

RUN cd /catkin_ws/src && \
    git clone https://github.com/srl-freiburg/pedsim_ros.git && \
    cd pedsim_ros && \
    git submodule update --init --recursive
RUN cd /catkin_ws/src && \
    git clone -b main https://github.com/reiniscimurs/DRL-robot-navigation.git
RUN cd /catkin_ws/src && \
    git clone https://github.com/leonhartyao/gazebo_models_worlds_collection.git

RUN pip3 install pyyaml rospkg attrs netifaces
ENV GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:/catkin_ws/src/gazebo_models_worlds_collection/worlds
ENV GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/catkin_ws/src/gazebo_models_worlds_collection/models
ENV LD_LIBRARY_PATH=/opt/hpcx/ompi/lib:$LD_LIBRARY_PATH
ENV NVIDIA_VISIBLE_DEVICES="all"
ENV NVIDIA_DRIVER_CAPABILITIES="all"
WORKDIR /catkin_ws/
RUN echo ". /usr/share/gazebo-11/setup.sh" >> ~/.bashrc
RUN echo ". /catkin_ws/devel/setup.bash" >> ~/.bashrc

RUN echo "ALL Done"