xhost local:root
XAUTH=/tmp/.docker.xauth
docker run --rm -it \
    --name=drl_container\
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --env="XAUTHORITY=$XAUTH" \
    --env="XDG_RUNTIME_DIR=/tmp/runtime-root"  \
    --volume="/home/hazel/catkin_ws/src/social_nav_drl:/catkin_ws/src/social_nav_drl" \
    --volume="/dev/bus/usb:/dev/bus/usb" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix" \
    --gpus all \
    --net=host \
    --privileged \
    --gpus all --shm-size=1g -it \
    drl_image_melodic \
    bash

echo "Done."