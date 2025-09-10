docker run -it --privileged \
    -e DISPLAY=$DISPLAY \
    --env="QT_X11_NO_MITSHM=1" \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v /dev:/dev:rw \
    --env=LOCAL_USER_ID="$(id -u)" \
    --hostname $(hostname) \
    --network host \
    --name your_container_name your_image:your_tag bash