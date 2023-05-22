#include <ros/ros.h>
#include <memory>

// MoveitCpp
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <geometry_msgs/PointStamped.h>

#include <chrono>

#include "control.hpp"


int main(int argc, char **argv){
    ros::init(argc, argv, "control_node"); 
    ros::NodeHandle nh("~"); 

    ros::AsyncSpinner spinner(1);
    spinner.start(); 

    control control(nh); 

    std::cout << "Class created" << std::endl;

    control.runAll();
    //control.approachTarget();


    std::cout << "Mission finished" << std::endl;

}