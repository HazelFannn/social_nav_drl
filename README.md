# TD3-Based Socially Aware Robot Navigation in Dynamic Environments

Autonomous navigation in dynamic environments has evolved beyond simple obstacle avoidance, necessitating an advanced understanding of social norms when robots and humans share the same space. This work introduces a robust navigation system leveraging the Twin Delayed Deep Deterministic Policy Gradient (TD3) algorithm, designed to navigate efficiently while respecting social norms. The proposed framework has been trained extensively in the Gazebo simulator with carefully designed simulation environments mimicking the dynamics of real-world settings.

## Getting Started

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes.

This project is set up to run within a Docker environment which ensures the environment consistency, it eliminates the "it works on my machine" problem by providing a uniform platform for development, testing, and production. A Docker container also wraps up an application with all the parts it needs, such as libraries and other dependencies, and ships it all out as one package. This means you can easily move the container from one computing environment to another.

## Installation

Clone the repository:
```shell
## Clone the repository
$ cd ~
$ git clone https://github.com/HazelFannn/social_nav_drl.git
```

## Using Docker

### About the Docker Container
This docker environment is based on the Nvidia PyTorch container Release 20.10. The container image contains the complete source of the version of PyTorch and it is pre-built and installed in Conda default environment. This container also includes the following:

* Ubuntu 18.04 including Python 3.6 environment
* NVIDIA CUDA 11.1.0 including cuBLAS 11.2.1
* NVIDIA cuDNN 8.0.4
* TensorBoard 1.15.0+nv

and more. The complete documentation can be found here: https://docs.nvidia.com/deeplearning/frameworks/pytorch-release-notes/rel_20-10.html

All docker-related files can be found here:
```bash
$ cd ~/catkin_ws/src/social_nav_drl/docker
```

Build the docker image if any modifications are made to the docker file:
```bash
$ bash build.bash
```

To run the docker container:
```bash
$ bash drl.bash
```

## Usage

After running the docker container, the default work directory is set to `/catkin_ws`. First, compile the workspace:
```shell
$ catkin_make
$ source devel/setup.bash
```

All simulation scenarios can be found here:
```shell
$ cd /catkin_ws/src/social_nav_drl/gazebo_sfm_plugins/worlds/
```

To change the simulation settings for training or testing:
```shell
$ cd /catkin_ws/src/social_nav_drl/DRL_Robot_Navigation/catkin_ws/src/
```

To run the training:
```shell
$ cd /catkin_ws/src/social_nav_drl/DRL_Robot_Navigation/TD3
$ python3 train_velodyne_td3.py
```