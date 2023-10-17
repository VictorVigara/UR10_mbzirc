#!/bin/bash
SHARED_DIR=${1:-$(pwd)}

xhost local:root

XAUTH=/tmp/.docker.xauth

docker run -it \
    --name ur10_mbzirc_develop \
    -e DISPLAY=$DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    --privileged \
	--network host \
	-v /tmp/.X11-unix:/tmp/.X11-unix:rw \
	-v $SHARED_DIR:/UR10_mbzirc_ws \
	-v $XAUTH:$XAUTH \
	-v /dev:/dev \
	--shm-size=512M \
	lucasmogsan/ur10_mbzirc_develop:latest