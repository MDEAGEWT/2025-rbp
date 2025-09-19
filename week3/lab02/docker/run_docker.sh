docker run -it --privileged \
    -e DISPLAY=$DISPLAY \
    --env="QT_X11_NO_MITSHM=1" \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v /dev:/dev:rw \
    -v /home/kmk/2025-rbp/week3/ros2_ws:/home/user/ros2_ws \
    --env=LOCAL_USER_ID="$(id -u)" \
    --hostname $(hostname) \
    --network host \
    --name rbp-lab02 mdeagewt/2025-rbp:lab01-assignment-answer bash
