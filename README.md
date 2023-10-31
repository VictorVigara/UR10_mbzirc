## TODO:
- Smth wrong with the make (due to camera...)... Camera (relsense2) is deleted and outcommented 

## UR10 workspace

The aim of this repository is to perform robotic perception and manipulation of target boxes. The workspace has several main packages: 

* ur_modern_driver: UR drivers to connect the robot to the computer through an ethernet cable. 
* ur_description: UR package that contains the URDF definitions of the UR10 robot that wil be used by MoveIt. 
* ur_msgs: Package that contains custo messages to control the UR. 
* moveit_calibration: Package used to calibrate the camera mounted on the UR10 giving the tf from the base_link. 
* custom_ur10_moveit_config: Package that launch the MoveIt environment with the UR10 robot to be used in control node. 
* control: Package that controls the UR10 through MoveIt commands. 
* perception: Package that runs camera node to detect target boxes and publish distance to it. 
* meta: Contain the general file to launch the whole mission. 

## Description

The goal of the mission is to detect target boxes and plan trajectories to grab them and pose them in the desired position. The workflow is as follows: 

1. Initial scan: A scan of the surroundings is performed to detect target boxes with the camera. 
2. Targer distance calculation: Once the target box is detected, the distance to it is calculated and published. 
3. Generate trajectory to grab the object: With the position of the target box, a trajectory is calculated to grab it avoiding possible obstacles. 
4. Grab the object. (Not implemented yet)
5. Generate final trajectory avoiding obstacles. 

The control of the manipulator has been done using the MoveIt package, which acts as an interface calculating trajectories given the desired point. Tutorials about this package can be found [here](https://ros-planning.github.io/moveit_tutorials/). In particular, the it has been used the [Move group C++ Interface](https://ros-planning.github.io/moveit_tutorials/doc/move_group_interface/move_group_interface_tutorial.html). 




 ## Docker
Make sure that Docker is installed following the official [Docker Installation Instructions](https://docs.docker.com/engine/install/). It is advisable to also follow the [Linux Post-installation steps](https://docs.docker.com/engine/install/linux-postinstall/).

0. Before starting up - on your local computer - go to your "wired connetion" and edit settings for your connection:
IPv4 -> method: manual -> adress: 192.38.66.1 - netmask 255.255.255.0

1. Pull the docker image:
```bash
./docker-pull.sh
```

2. Run the docker container specifiying a shared folder path between the host and the container:
```bash
./docker-run.sh /path/to/shared/folder
```

if you are already in your /UR10_MBZIRC folder just type:
```bash
./docker-run.sh .
```

3. To open more terminals in the running container:

```bash
docker exec -it ur10_mbzirc_develop bash
```

4. To restart an stopped container after a PC reboot (then do exec):

```bash
docker start ur10_mbzirc_develop
```

## Building the image (if needed)
Build image
1. cd to the folder where the dockerfile is located (in this repos root folder)
```bash
docker build . -t lucasmogsan/ur10_mbzirc_develop:latest
```
2. The following is optional if you want to push to dockerhub:
```bash
docker tag lucasmogsan/ur10_mbzirc_develop:latest docker.io/lucasmogsan/ur10_mbzirc_develop:latest
```

```bash
docker push docker.io/lucasmogsan/ur10_mbzirc_develop:latest
```



## Runningt the code
First source and build the ros environment and packages.
```
$ (maybe needed) source /opt/ros/noetic/setup.bash
$ cd /UR10_mbzirc_ws
$ catkin_make
$ source devel/setup.bash
```
catkin_make might take a while, 5-10 min, the first time


Make sure to change ***ur10_bringup.launch*** in ***ur_modern_driver*** to your own ip adress (should be changed 192.38.66.1)
- Test by ping the robot (eg. ping 192.38.66.227)


**Run test:** to see controller is working. Manually move the robot around in rviz:
```
$ roslaunch ur_modern_driver ur10_bringup.launch
$ roslaunch custom_ur10_moveit_config demo_ur10.launch
```

Test for sensor:
```
$ roslaunch vl53l0x_driver test.launch
```


**Run main program:** to find and pick the box object:
```
$ roslaunch meta camera_mission.launch
```

## Modifying code -using camera or dist-sensor:

If distance sensor is being used:
- set control-config dist_senor to True
- set meta - launch - camera_mission.launch dist_sensor_on to true

**Fixed obstacles / objects**




## Requirements
This workspace works in Ubuntu 20.04 with ROS Noetic and the UR10 CB2 version. 
To be able to use the moveit package, this need to be installed:  
```
$ sudo apt install ros-noetic-moveit 
```
 
## Installation
Clone the repository into your catkin working directory and initialize submodules: 
```
$ git clone https://github.com/VictorVigara/UR10_mbzirc
$ git submodule init
$ git submodule update 
```
and make the workspace: 
```
$ catkin_make
```

## Usage

### UR10 ethernet configuration
The connection to the UR10 is done through ethernet. Once the ethernet is connected to the computer, the ethernet configuration is as following:
* IPv4 method: Manual
* Address: 192.38.66.1
* Netmask: 255.255.255.0
* Gateway: 

Then, your computer IP need to be set in the ur_modern_driver package. In the launch file "ur_modern_driver/launch/ur10_bringup.launch" set the reverse_ip parameter to your computer IP, to let the robot read the trajectory sent by your computer. 


### Camera calibration
Camera calibration should be done when placing the camera into the robotic arm. For that purpose, the moveit_calibration package will be used. First of all, the camera calibration launch file have to be launched: 
```
$ roslaunch meta camera_calibration.launch
```
Rviz should show the real state of the UR10. If that is the case, the calibration is done following the steps in this [moveit calibration tutorial](https://ros-planning.github.io/moveit_tutorials/doc/hand_eye_calibration/hand_eye_calibration_tutorial.html). 
Once the tutorial has been completed, the launch file generated that contains the transform between the camera and the UR base link need to be placed in src/perception/launch with the name tf_camera.launch. 

### Mission parameters
A config file has been created in control package (src/control/config/params.yaml) with the following parameters: 
* manual: [TRUE/FALSE] If TRUE, the robot won't move until enter key is pressed for every trajectory calculated. 
* static_scan_time: [>0] Time the robot is stopped in the scan position to let the camera scan the environment. 
* vel_scaling_factor: [0-1] Factor to reduce the max velocity.
* acc_scaling_factor: [0-1] Factor to reduce the max acceleration.
* real_camera: [TRUE/FALSE] If False, dummy camera node simulates the camera function pulbishing a target_position. 


### Launch mission

A meta package has been created to launch the required nodes to start the mission: 

```
$ roslaunch meta camera_mission.launch
```

Inside the launch file ther is an argument which is real_camera, that can be set to false when no camera is connected. In this mode, a dummy camera node will be launched simulating the target position published by the camera. 

This general launch file launches: 
1) UR10 driver (ur_modern_driver)
2) MoveIt UR10 configuration (custom_ur10_moveit_config)
3) Perception launch file (perception) 
4) Control launch file (control)

which would perform the mission described in Description. 

