# 如何为GPU搭建一个docker环境
## 1. 前言
之前我一直以为 `docker` 使用 `GPU` 是很直接的事情, 不需要配置什么, 所以尝试直接在之前的 `container` 里面安装 `pytorch`, 但是一直遇到找不到 `GPU` 的问题, 随后才开始考虑这个问题.
通过阅读下面[这个教程](https://docs.nvidia.com/deeplearning/frameworks/user-guide/index.html), 我注意到两个关键点:
- 我是可以直接下载装好 `Nvidia` 驱动的 `images` 的, 只不过这些 `images` 并不是发布到了 `Docker hub`, 而是 `Nvidia` 自己的[一个网站](https://catalog.ngc.nvidia.com/containers).
- 像使用其他设备一样, 即使安装好了驱动, 我们在启动 `container` 的时候仍然需要将主机的 `GPU` 设备映射给 `container` 才行.
## 2. 解决方案
### 2.1 方案
依据前言, 理论上来讲, 我现在下载了官方安装好驱动的 `image`, 然后将 `GPU` 挂载上去就可以了. 这个方案正在进行尝试.
我使用如下 `Dockerfile` 进行构建名为 `docker_gpu`的 `image`:
```dockerfile
FROM nvcr.io/nvidia/ai-workbench/pytorch:1.0.2
RUN apt-get update
RUN apt-get install -y octave
```
Use this command to build the docker image:
`docker build . -t docker_gpu`
构建成功之后, 使用下面的命令启动 `container`:
```bash
xhost local:root
XAUTH=/tmp/.docker.xauth
docker run --rm -it \
    --name=docker_gpu_container\
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --env="XAUTHORITY=$XAUTH" \
    --volume="/dev/bus/usb:/dev/bus/usb" \
    --gpus all \ # 这一句是用来挂载GPU到container的,不添加它的话是检测不到GPU的
    --net=host \
    --privileged \
    docker_gpu \
    bash
echo "Done."
```
### 2.2 结果
进入之后, 直接运行下面的指令, 输出为`True`, 说明torch已经成功安装了且可以使用`GPU`版本.
```bash
root@amnl-XPS-8930:/opt/hpcx/ucx/lib# python3
Python 3.10.12 (main, Jun 11 2023, 05:26:28) [GCC 11.4.0] on linux
Type "help", "copyright", "credits" or "license" for more information.
>>> import torch
>>> torch.cuda.is_available()
True
>>>
```
<details> <summary>一个小错误</summary>
中间在导入`torch`的时候遇到一个小错误:
```bash
>>> import torch
Traceback (most recent call last):
  File "<stdin>", line 1, in <module>
  File "/usr/local/lib/python3.10/dist-packages/torch/__init__.py", line 234, in <module>
    from torch._C import *  # noqa: F403
ImportError: /opt/hpcx/ucx/lib/libucs.so.0: undefined symbol: ucm_set_global_opts
```
通过设置环境变量解决:
```bash
export LD_LIBRARY_PATH=/opt/hpcx/ucx/lib:$LD_LIBRARY_PATH
```
</details>

## 2. 一些原理
![Alt text](image.png)## 找到container
- 可以通过[这个链接](https://catalog.ngc.nvidia.com/containers)来找到已经构建好学习环境的`dokcer nvidia containers`.
- 可以通过[这个链接](https://docs.nvidia.com/deeplearning/frameworks/pytorch-release-notes/index.html)来查看不同的container的系统信息和包含的软件。
> :memo: **Note**
>
> `container` 结尾为`.1`的好像是基于 `Rockey linux` 的:
> - `18.12-py3`: 查阅[链接](https://docs.nvidia.com/deeplearning/frameworks/pytorch-release-notes/index.html)以及实测后确认为基于 `ubuntu 16.04`的 `container`.
> - `18.12.1-py3`: 实测为基于 `Rockey linux` 的 `container`.
> - `18.12.1-py3`: 实测为基于 `Rockey linux` 的 `container`.