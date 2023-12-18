### Build the image
```
$ cd <the_folder_you_saved_files>
$ chmod +x *
$ docker build -t ros-noetic-conda .
```
### Build the volume
```
docker create volume <volume_name>
```
### Build the mount
Build a folder for bind mount and replace the source folder path in bash file `--mount` with the path of your new built folder
### Build the container
```
$ cd <the_folder_you_saved_files>
&./noetic_docker.bash
```
### Use the Ros UR control
In the container:
```
$ conda create -y -n grasp_experiments python=3.9
$ conda activate grasp_experiments 
$ pip install transforms3d 
$ pip install rospkg
$ pip install defusedxml
$ pip install pyserial
$ pip install pymodbus==2.2.0
$ conda install -y -c conda-forge python-orocos-kdl
$ roslaunch grasp_experiments UR_robot.launch
```
Keep this terminal running, open another terminal and:
```
$ docker container ls
```
copy the container ID
```
$ docker exec -it <container_id>
$ cd /catkin_ws/src/grasp_experiments/src
$ python3 main.py
```

docker run --rm -t -d --name drl_container -e DISPLAY=host.docker.internal:0 -v "C:\Users\hazel\Documents\MEng Project\TD3_Navigation\:/home" -it ros-noetic-nvidia
docker exec -it <id> bash

cd home/catkin
catkin_make_isolated
export GAZEBO_RESOURCE_PATH=/home/catkin_ws/src/multi_robot_scenario/launch
source devel_isolated/setup.bash
python3 train_velodyne_td3.py

killall -9 rosout roslaunch rosmaster gzserver nodelet robot_state_publisher gzclient python python3

docker run --rm --gpus all --privileged -v /dev:/dev nvidia/cuda:12.3-base nvidia-smi