// ROS
#include <ros/ros.h>
#include <ros/package.h>

#include <std_msgs/String.h>
#include <geometry_msgs/PointStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Range.h>

#include <sstream>
#include <memory>
#include <iostream>

// MoveitCpp
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include "control.hpp"

using namespace std;


// Constructor definition for control class
control::control(ros::NodeHandle& nodehandle):nh_(nodehandle), move_group_interface_arm("ur10_arm"), tfListener(tf_buffer){
    
    ROS_INFO("Constructing control class...");

    // Subscribe to topics and create publishers
    subscribeToTopics();
    createPublishers();
    loadParameters();

    // Set maximum velocity and acceleration scaling factor
    move_group_interface_arm.setMaxVelocityScalingFactor(vel_scaling_factor); 
    move_group_interface_arm.setMaxAccelerationScalingFactor(acc_scaling_factor); 

    // Initialize variables
    target_received = false;
    grab_object = false;
    scan_finished = false;
    mission_finished = false;
}

void control::subscribeToTopics(){
    targetPoseSubscriber_ = nh_.subscribe("/target_pose", 1, &control::targetPoseCallback, this);
    // TODO: implement distance sensor subscriber
    distsensorsubscriber_ = nh_.subscribe("/range_data", 1, &control::distSensorCallback, this);
}

void control::createPublishers(){
    scanFinishedPublisher = nh_.advertise<std_msgs::Bool>("/mission_status", 1000);
}

geometry_msgs::Point control::transform_between_frames(geometry_msgs::Point p, const std::string from_frame, const std::string to_frame) {
    
  geometry_msgs::PoseStamped input_pose_stamped;
  input_pose_stamped.pose.position = p;
  input_pose_stamped.header.frame_id = from_frame;
  input_pose_stamped.header.stamp = ros::Time::now();

  geometry_msgs::PoseStamped output_pose_stamped = tf_buffer.transform(input_pose_stamped, to_frame, ros::Duration(1));
  return output_pose_stamped.pose.position;
}

void control::attachBox(){
    target_box_object.id = "small_box";             // Id of the object to be attached
    target_box_object.header.frame_id = "ee_link";  // Frame id to be attached (frame with which the box is fixed)

    // Define box dimensions
    box_primitive.type = box_primitive.BOX; 
    box_primitive.dimensions.resize(3); 
    box_primitive.dimensions[0] = 0.1;
    box_primitive.dimensions[1] = 0.3;
    box_primitive.dimensions[2] = 0.2;

    // Define the position of the box wrt the frame id with which has been attached before
    orientation.setRPY(0, 0, 0);
    grab_pose.orientation = tf2::toMsg(orientation);
    grab_pose.position.x = 0.1; 

    target_box_object.primitives.push_back(box_primitive); 
    target_box_object.primitive_poses.push_back(grab_pose); 
    target_box_object.operation = target_box_object.ADD;
    planning_scene_interface.applyCollisionObject(target_box_object);
    move_group_interface_arm.attachObject(target_box_object.id, "ee_link");
    std::cout << "Object attached virtually to avoid obstacles with it" << std::endl;
}

void control::targetPoseCallback(const geometry_msgs::Pose::ConstPtr &msg){

    // Read target position in camera frame
    target_pose_camera_frame = msg->position;
    //std::cout << "Target camera frame: " << target_pose_camera_frame << std::endl;

    // Obtain target position in robot frame
    target_pose_robot_frame = transform_between_frames(target_pose_camera_frame, camera_frame, base_link_frame);
    //std::cout << "Target robot frame: " << target_pose_robot_frame << std::endl;

    // Update target box position 
    target_x = target_pose_robot_frame.x; 
    target_y = target_pose_robot_frame.y; 
    target_z = target_pose_robot_frame.z; 

    target_received = true;
}


void control::distSensorCallback(const sensor_msgs::Range& range_msg){
    if (range_msg.range > 1.5 or range_msg.range == 0.0){
        std::cout << "Object not in range" << std::endl;
    }
    else {
        sensor_z = range_msg.range;
        std::cout << "Distance to object [m]: " << sensor_z << std::endl;
    }
}
/*


void control::followObject(){
    // TODO: implement follow object
    std::cout << "Follow object" << std::endl;

    z_distance_list = [];   // List to store the last measurements of the distance sensor
    n_measurements = 0;     // Number of measurements to calculate the z-velocity of the object
    // Initialize the list with readings from the distance sensor
    for (int i = 0; i < n_measurements; i++){
        z_distance_list.append(sensor_z);
    }

    // Initialize flag to check if the target is close enough to grab it
    ready_to_grab = false;

    while not ready_to_grab{
        // Read target position from distance sensor:
        z_distance = sensor_z;

        // TODO: Calculate the z-velocity of the object (using the prev 5 measurements)
        // Append the new measurement to the list and if the list has more than 5 elements, remove the first one
        z_distance_list.append(z_distance);
        if (z_distance_list.size() > n_measurements){
            z_distance_list.pop(0);
        }
        z_velocity = (z_distance_list[n_measurements-1] - z_distance_list[0])/n_measurements;

        

        // Set robot target pose:
        // Downwards orientation per default
        orientation.setRPY(-(2* M_PI)/4, 0, 0);
        target_pose.orientation = tf2::toMsg(orientation);
        // Fill target_pose with the position received from the camera node transformed to robot coordinates
        target_pose.position.x = target_x;
        target_pose.position.y = target_y;
        // Make a poistion control to follow the object:
        target_pose.position.z = target_z - z_distance + 0.15;

        // Update target pose
        move_group_interface_arm.setPoseTarget(target_pose);

        // Plan and move 
        planAndMove();

        // Check if the target is close enough to grab it
        if (z_distance < 0.2 and z_distance > 0.0){
            ready_to_grab = true;
        }
    }

}

*/

void control::initialScan(){
    // Implement a dynamic scan to find the target box
    std::cout << "Moving to scan position ..." << std::endl;

    // Joint values to initial scan
    /* std::vector<double> home_joints = {-1.5708, -1.5708, -0.698132, -2.26893, 1.5708, 0};
    move_group_interface_arm.setJointValueTarget(home_joints); 
    current_pose = move_group_interface_arm.getCurrentPose("ee_link");    
    planAndMove(); */

    orientation.setRPY(-(2* M_PI)/4, 0, 0);
    target_pose.orientation = tf2::toMsg(orientation);
    target_pose.position.x = 0; 
    target_pose.position.y = 0.5;
    target_pose.position.z =  1;

    move_group_interface_arm.setPoseTarget(target_pose);
    planAndMove();
    ros::Duration(static_scan_time).sleep();

    scan_finished = true;

    // Publish message to start target detection
    std_msgs::Bool msg1;
    msg1.data = true;
    scanFinishedPublisher.publish(msg1);
}

void control::approachTarget(){
    
    std::cout << "Target recieved" << std::endl;

    // Downwards orientation per default
    orientation.setRPY(-(2* M_PI)/4, 0, 0);
    target_pose.orientation = tf2::toMsg(orientation);

    // Fill target_pose with the position received from the camera node transformed to robot coordinates
    target_pose.position.x = target_x;
    target_pose.position.y = target_y;
    target_pose.position.z = target_z + 0.15;

    // Update target pose
    move_group_interface_arm.setPoseTarget(target_pose);

    // Plan and move 
    planAndMove();

    std::cout << "Target approached" << std::endl;

}


void control::grabObject(){

    // Insert code to grab the object with the gripper
    std::cout << "Grabbing object" << std::endl;

    // Attach box to the ee to plan and avoid obstacles with it
    attachBox();

    std::cout << "Target grabbed" << std::endl;


    std::cout << "Moving out ..." << std::endl;
    current_pose = move_group_interface_arm.getCurrentPose("ee_link");  
    std::cout << current_pose << std::endl; 
    target_pose.position = current_pose.pose.position; 
    target_pose.position.z = current_pose.pose.position.z + 0.5;
    std::cout << target_pose << std::endl;
    move_group_interface_arm.setPoseTarget(target_pose);

    // Plan the trajectory and move to it
    planAndMove();

}

void control::goFinalPosition(){
    
    std::cout << "Planning to final position" << std::endl;

    // Set final position coordinates
    orientation.setRPY(-(2* M_PI)/4, 0, 0);
    target_pose.orientation = tf2::toMsg(orientation);
    target_pose.position.x = -0.4;
    target_pose.position.y = 0.5;
    target_pose.position.z = 0.2;
    move_group_interface_arm.setPoseTarget(target_pose);

    // Plan the trajectory and move to it
    planAndMove();

    std::cout << "Final position reached" << std::endl;

    move_group_interface_arm.detachObject(target_box_object.id);
}


void control::testSequence(){
    std::cout << "Planning pos 1" << std::endl;

    for (int i = 0; i < 5; i++){

        std::cout << "Planning pos 1" << std::endl;
        orientation.setRPY(-(2* M_PI)/4, 0, 0);
        target_pose.orientation = tf2::toMsg(orientation);
        target_pose.position.x = 0; 
        target_pose.position.y = 0.5;
        target_pose.position.z =  0.4;

        move_group_interface_arm.setPoseTarget(target_pose);
        planAndMove();
        ros::Duration(5).sleep();

        std::cout << "Planning pos 2" << std::endl;

        orientation.setRPY(-(2* M_PI)/4, 0, 0);
        target_pose.orientation = tf2::toMsg(orientation);
        target_pose.position.x = 0; 
        target_pose.position.y = 0.6;
        target_pose.position.z =  0.2;

        move_group_interface_arm.setPoseTarget(target_pose);
        planAndMove();
        ros::Duration(5).sleep();
    }

}

void control::runAll(){

    testSequence();

    // Initial scan to detect target object
    initialScan(); 

    // Wait until the target position has been published by the camera node
    while (ros::ok && grab_object == false){
        if (target_received == true && grab_object == false){
            // Move to target position received from camera
            approachTarget(); 

            // Update grab_object flag to start the grabbing 
            grab_object = true;
        }
        // Update callback values
        ros::spinOnce();
    }

    // Grab object (Gripper control has to be implemented)
    grabObject(); 

    // Plan and move to the final position with the box
    goFinalPosition(); 
}


void control::planAndMove(){
    success = (move_group_interface_arm.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (manual == true){
        cout << "Press Enter to continue...";
        fflush(stdin); // clear input buffer
        getchar(); // wait for user to press Enter
    }
    move_group_interface_arm.move();
}

void control::loadParameters(){
    /* if (!nh_.param<std::string>("target_pose_topic", target_pose_topic, "/target_pose")) {
        ROS_WARN_STREAM("Did not found target_pose_topic name. Setted to a standard value: " << target_pose_topic);
    }

    if (!nh_.param<std::string>("scan_finished_topic", scan_finished_topic, "/mission_status")) {
        ROS_WARN_STREAM("Did not found scan_finished_topic name. Setted to a standard value: " << scan_finished_topic);
    }*/

    if (!nh_.param<bool>("manual", manual, true)) {
        ROS_WARN_STREAM("Did not found manual name. Setted to a standard value: " << manual);
    } 

    if (!nh_.param<int>("static_scan_time", static_scan_time, 2)) {
        ROS_WARN_STREAM("Did not found static_scan_time name. Setted to a standard value: " << static_scan_time);
    }

    if (!nh_.param<double>("vel_scaling_factor", vel_scaling_factor, 0.1)) {
        ROS_WARN_STREAM("Did not found vel_scaling_factor name. Setted to a standard value: " << vel_scaling_factor);
    }

    if (!nh_.param<double>("acc_scaling_factor", acc_scaling_factor, 0.1)) {
        ROS_WARN_STREAM("Did not found acc_scaling_factor name. Setted to a standard value: " << acc_scaling_factor);
    } 
}
