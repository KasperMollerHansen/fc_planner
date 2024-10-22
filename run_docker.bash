xhost local:root

docker run -it \
    -v /home/$USER/:/home/$USER/ \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -e DISPLAY=$DISPLAY \
    -e QT_X11_NO_MITHSHM=1 \
    --rm \
    --name docker_ws \
    --network host \
    --privileged \
    docker_ws \
    bash

echo "Done."

