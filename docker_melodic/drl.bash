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
    --gpus all \
    --net=host \
    --privileged \
    drl_image_melodic \
    bash

echo "Done."