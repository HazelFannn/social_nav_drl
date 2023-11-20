FROM osrf/ros:noetic-desktop-full


#system general setup
RUN apt-get update && \
    apt-get -y install ros-noetic-catkin&& \
    apt-get -y upgrade && \
    apt-get install wget && \
    apt-get install nano && \
    apt-get install -y git && apt-get install -y python3-pip && \
    apt install -y ros-noetic-moveit && \
    apt-get install -y ros-noetic-gazebo-ros-control && \
    apt-get install -y ros-noetic-ros-control && \
    apt-get install -y x11-apps && \
    apt-get install -y ros-noetic-industrial-robot-status-controller && \
    apt-get install -y iputils-ping && \
    mkdir -p /catkin_ws/src

# controller install
RUN apt-get update && \
    apt-get install ros-noetic-joint-state-controller && \
    apt-get install -y ros-noetic-effort-controllers && \
    apt-get install ros-noetic-position-controllers && \
    # apt-get install -y ros*controller* && \
    apt install ros-noetic-joint-state-publisher-gui

# gazebo setup
RUN mkdir -p ~/.gazebo && \
    cd ~/.gazebo && \
    git clone https://github.com/osrf/gazebo_models.git && \
    mv gazebo_models models

# trac-ik setup
RUN apt-get update && \
    apt-get install -y ros-noetic-trac-ik-kinematics-plugin && \
    apt-get -y upgrade && \
    /bin/bash -c '. /opt/ros/noetic/setup.bash; cd /catkin_ws; catkin_make;. ./devel/setup.bash'

# ROS UR driver setup
RUN cd catkin_ws/src && \
    git clone -b melodic-devel https://github.com/ros-industrial/universal_robot.git && \
    git clone https://github.com/UniversalRobots/Universal_Robots_ROS_Driver.git && \
    cd .. && \
    apt update -qq && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -y && \
    /bin/bash -c '. /opt/ros/noetic/setup.bash; cd /catkin_ws; catkin_make;. ./devel/setup.bash'
    # cd universal_robot/ur_description/urdf
    # rm common.gazebo.xacro && \
    # git clone https://github.com/JensenZhaoUT/modified_files.git

# intel realsense camera setup 
RUN mkdir -p /etc/apt/keyrings && \
    apt-get update && \
    apt-get install -y curl && \
    curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null && \
    echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo `lsb_release -cs` main" | \
    tee /etc/apt/sources.list.d/librealsense.list && \
    apt-get update && \
    apt-get install -y apt-transport-https && \
    apt-get update && \
    apt-get install -y librealsense2-dkms && \
    apt-get install -y librealsense2-utils && \
    apt install -y ros-noetic-librealsense2* && \
    apt-get install -y ros-noetic-realsense2-camera

# robotiq gripper setup
RUN cd catkin_ws/src && \
    git clone https://github.com/TAMS-Group/robotiq.git

RUN apt-get update && \
    /bin/bash -c '. /opt/ros/noetic/setup.bash; cd /catkin_ws; catkin_make;. ./devel/setup.bash; rosdep install -y robotiq_modbus_tcp' && \
    apt-get install -y ros-noetic-soem && \
    apt-get install ros-noetic-socketcan-interface 

# conda environment setup
RUN pip install pyserial && \
    pip install pymodbus==2.2.0 && \
    pip install transforms3d

# RUN /bin/bash -c '. /opt/ros/noetic/setup.bash; cd /catkin_ws; rosdep update; rosdep install --from-paths src --ignore-src -y; catkin_make;. ./devel/setup.bash'

RUN echo ". /opt/ros/noetic/setup.bash" >> ~/.bashrc
RUN echo ". /catkin_ws/devel/setup.bash" >> ~/.bashrc

#conda install
RUN wget --quiet https://repo.anaconda.com/archive/Anaconda3-2019.07-Linux-x86_64.sh -O ~/anaconda.sh\
    && /bin/bash ~/anaconda.sh -b -p "/opt/conda" \
    && rm ~/anaconda.sh 
RUN apt-get -y upgrade

# Make conda command available to all users
RUN ln -s /opt/conda/etc/profile.d/conda.sh /etc/profile.d/conda.sh

# Activate conda environment with interactive bash session
RUN echo ". /opt/conda/etc/profile.d/conda.sh" >> ~/.bashrc
RUN /bin/bash -c '. /opt/conda/etc/profile.d/conda.sh; conda activate; pip install transforms3d; pip install rospkg; pip install defusedxml; pip install pyserial; pip install pymodbus==2.2.0'
RUN echo "ALL Done"

#some common commands to use docker
#docker run -t -d -v myVolume:/ur_ws/src <image_name>
#docker exec -it <container_id> bash
#docker image rm
#docker build -t <image_name> .
#docker volume create <volume_name>
