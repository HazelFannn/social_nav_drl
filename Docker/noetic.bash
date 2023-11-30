docker run --rm -it \
    --name=drl_container\
    -e DISPLAY=host.docker.internal:0.0
    --volume="C:\Users\hazel\Documents\MEng Project\TD3_Navigation:/catkin_ws/src/TD3_Navigation" \
    --volume="/dev/bus/usb:/dev/bus/usb" \
    --net=host \
    --privileged \
    ros-noetic \
    bash
docker run --rm -t -d --name drl_container -e DISPLAY=host.docker.internal:-1 -v "C:\Users\hazel\Documents\MEng Project\TD3_Navigation\:/home" -it ros-noetic
docker exec -it <id> bash