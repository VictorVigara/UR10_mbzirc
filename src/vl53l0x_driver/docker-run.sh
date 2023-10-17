#!/bin/bash
SHARED_DIR=${1:-$(pwd)}

xhost local:root

XAUTH=/tmp/.docker.xauth

docker run -it \
    --name vl53l0x_driver \
    -e DISPLAY=$DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    --privileged \
	--network host \
	-v /tmp/.X11-unix:/tmp/.X11-unix:rw \
	-v $SHARED_DIR:/my_ws/src/vl53l0x_driver_ws \
	-v $XAUTH:$XAUTH \
	-v /dev:/dev \
	--shm-size=512M \
	lucasmogsan/vl53l0x_driver:latest