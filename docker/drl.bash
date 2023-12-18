xhost local:root
XAUTH=/tmp/.docker.xauth
docker run --rm -it \
    --name=drl_container\
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --env="XAUTHORITY=$XAUTH" \
    --volume="/home/hazel/catkin_ws/src/social_nav_drl:/catkin_ws/src" \
    --volume="/dev/bus/usb:/dev/bus/usb" \
    --net=host \
    --privileged \
    drl_image \
    bash

echo "Done."