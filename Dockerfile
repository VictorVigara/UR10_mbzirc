FROM osrf/ros:noetic-desktop

## Install dependencies:
RUN apt-get update
RUN apt-get install -y python3-pip \
                    git

RUN apt-get install -y ros-noetic-moveit \
                    ros-noetic-moveit-visual-tools \
                    ros-noetic-ddynamic-reconfigure \
                    ros-noetic-image-geometry \
                    ros-noetic-rqt-graph \
                    ros-noetic-rviz-visual-tools \
                    ros-noetic-robot-state-publisher \
                    ros-noetic-controller-manager \
                    ros-noetic-industrial-msgs \
                    ros-noetic-pcl-ros
                    
RUN apt-get update
RUN apt-get install -y xorg-dev \
                    libglu1-mesa-dev \
                    libssl-dev \
                    libusb-1.0-0-dev \
                    libudev-dev pkg-config \
                    libgtk-3-dev \
                    cmake \
                    net-tools \
                    iputils-ping \
                    curl \
                    zip \
                    unzip \
                    tar \
                    dh-autoreconf



## Install realsense stuff
RUN git clone https://github.com/Microsoft/vcpkg.git
RUN cd /vcpkg && \
    ./bootstrap-vcpkg.sh && \
    ./vcpkg integrate install && \
    ./vcpkg install realsense2 && \ 
    ./vcpkg update
    # can remove folder and just keep installation ??
    #cd .. && \
    #rm -rf /vcpkg
    

RUN git clone https://github.com/IntelRealSense/librealsense.git
RUN cd /librealsense && \
    mkdir build && \
    cd build && \
    cmake .. && \
    make -j $(nproc) && \
    make install && \
    # can remove folder and just keep installation
    cd .. && \
    rm -rf /librealsense





# To make the docker image
# docker build . -t lucasmogsan/ur10_mbzirc:latest
# docker tag lucasmogsan/ur10_mbzirc:latest docker.io/lucasmogsan/ur10_mbzirc:latest
# docker push docker.io/lucasmogsan/ur10_mbzirc:latest