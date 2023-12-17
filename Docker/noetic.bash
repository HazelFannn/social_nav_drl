docker run --rm -it \
    --name=drl_container\
    -e DISPLAY=host.docker.internal:0.0
    --volume="C:\Users\hazel\Documents\MEng Project\TD3_Navigation:/catkin_ws/src/TD3_Navigation" \
    --volume="/dev/bus/usb:/dev/bus/usb" \
    --net=host \
    --privileged \
    ros-noetic \
    bash
docker run --rm -t -d --name drl_container -e DISPLAY=host.docker.internal:0 -v "C:\Users\hazel\Documents\MEng Project\TD3_Navigation\:/home" -it ros-noetic-nvidia
docker exec -it <id> bash

cd home/catkin
catkin_make_isolated
export GAZEBO_RESOURCE_PATH=/home/catkin_ws/src/multi_robot_scenario/launch
source devel_isolated/setup.bash
python3 train_velodyne_td3.py

killall -9 rosout roslaunch rosmaster gzserver nodelet robot_state_publisher gzclient python python3

docker run --rm --gpus all --privileged -v /dev:/dev nvidia/cuda:12.3-base nvidia-smi