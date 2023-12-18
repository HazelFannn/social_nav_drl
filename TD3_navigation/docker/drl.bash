docker run --rm -it \
    --name=drl_container\
    -e DISPLAY=host.docker.internal:0.0
    --volume="/home/hazel/catkin_ws/src/social_nav_drl:/catkin_ws/src/" \
    --volume="/dev/bus/usb:/dev/bus/usb" \
    --net=host \
    --privileged \
    drl \
    bash