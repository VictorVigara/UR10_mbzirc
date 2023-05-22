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

### Camera calibration
Camera calibration should be done when placing the camera into the robotic arm. For that purpose, the moveit_calibration package will be used. First of all, the camera calibration launch file have to be launched: 
```
$ roslaunch meta camera_calibration.launch
```
Rviz should show the real state of the UR10. If that is the case, the calibration is done following the steps in this [moveit calibration tutorial](https://ros-planning.github.io/moveit_tutorials/doc/hand_eye_calibration/hand_eye_calibration_tutorial.html). 
Once the tutorial has been completed, the launch file generated that contains the transform between the camera and the UR base link need to be placed in src/perception/launch with the name tf_camera.launch. 



### Launch mission

A meta package has been created to launch the required nodes to start the mission: 

```
$ roslaunch meta camera_mission.launch
```

This general launch file launches: 
1) UR10 driver (ur_modern_driver)
2) MoveIt UR10 configuration (custom_ur10_moveit_config)
3) Perception launch file (perception) 
4) Control launch file (control)

which would perform the mission described in Description. 

