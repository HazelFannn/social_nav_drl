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