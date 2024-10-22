xhost local:root

docker run -it \
    -v /home/$USER/:/home/$USER/ \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -e DISPLAY=$DISPLAY \
    -e QT_X11_NO_MITHSHM=1 \
    --rm \
    --name fc \
    --network host \
    --privileged \
    fc_planner \
    bash

echo "Done."

