// ROS
#include <ros/ros.h>
#include <ros/package.h>

#include <std_msgs/String.h>
#include <geometry_msgs/PointStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Range.h>

#include <sstream>
#include <memory>
#include <iostream>
#include <list>

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


#include <fstream>
#include <vector>
#include <geometry_msgs/Point.h>


// TODO: Attach tool to the robot to avoid obstacles with it
// TODO: Make the static objects match the boat (when known)



// Constructor definition for control class
control::control(ros::NodeHandle& nodehandle)
    : nh_(nodehandle),
    move_group_interface_arm("ur10_arm"),
    tfListener(tf_buffer),
    pose_and_velocity_history()
{
    
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
    z_distance_list = {};   // List to store the last measurements of the distance sensor
}

void control::subscribeToTopics(){
    targetPoseSubscriber_ = nh_.subscribe("/target_pose", 1, &control::targetPoseCallback, this);
    control_effort_subscriber = nh_.subscribe("/control_effort", 1, &control::controlEffortCallback, this);
    distsensorsubscriber_ = nh_.subscribe("/range_data", 1, &control::distSensorCallback, this);
}

void control::createPublishers(){
    scanFinishedPublisher = nh_.advertise<std_msgs::Bool>("/mission_status", 1000);
    speedlPublisher = nh_.advertise<std_msgs::String>("/ur_driver/URScript", 1000);
    setpoint_pid = nh_.advertise<std_msgs::Float64>("/setpoint", 1000);
    state_pid = nh_.advertise<std_msgs::Float64>("/state", 1000);
    pid_enable = nh_.advertise<std_msgs::Bool>("/pid_enable", 1000);
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


double sum(std::list<double>& list) {
    double sum = 0;
    for(double num : list) {
        sum += num;
    }
    return sum;
}

void control::distSensorCallback(const sensor_msgs::Range& range_msg){
    // Read distance sensor
    if (range_msg.range > 1.5 or range_msg.range < 0.001){
        // If the object is not in range, set the distance to a fixed value (max read distance)
        sensor_z = 1.5;       
    }
    else {
        // If the object is in range, read the distance and store it in a list to calculate the mean
        sensor_z = range_msg.range;
        if (z_distance_list.size() < 3){
            z_distance_list.push_back(sensor_z);
        }
        else{
            z_distance_list.pop_front();
            z_distance_list.push_back(sensor_z);
        }
        // Calculate the mean distance of the last 3 measurements
        sensor_z_mean = sum(z_distance_list)/z_distance_list.size();
    } 
}


void control::controlEffortCallback(const std_msgs::Float64& control_effort_msg){
    output_pid = control_effort_msg.data;
    //std::cout << "Control effort: " << output_pid << std::endl;
}



void control::followObject(){
    std::cout << "Follow object" << std::endl;

    // Initialize flag to check if the target is close enough to grab it
    bool move_to_grab = false;
    bool grab_object = false;
    int counter_follow_object = 0;
    
    // Distance to follow the object and distance to grab it
    float follow_dist = 0.3;
    float grab_dist = 0.03;
    std_msgs::String speedl_msg;

    // Initialize position above target
    // TODO: Read x,y from camera node
    std::cout << "Planning pos 1" << std::endl;
    orientation.setRPY(-(2* M_PI)/4, 0, 0);
    target_pose.orientation = tf2::toMsg(orientation);
    target_pose.position.x = -0.07; 
    target_pose.position.y = 0.55;
    target_pose.position.z =  0.8;
    move_group_interface_arm.setPoseTarget(target_pose);
    planAndMove();
    ros::Duration(3).sleep();

    // Restart PID controller (to avoid any old values):
    double z_robot_vel = 0;
    std_msgs::Bool enable_msg;
    state_pid.publish(0);
    setpoint_pid.publish(0);
    ros::Duration(0.2).sleep();
    enable_msg.data = false;
    pid_enable.publish(enable_msg);
    ros::Duration(0.2).sleep();
    enable_msg.data = true;
    pid_enable.publish(enable_msg);
    // Setpoint for PID controller:
    std_msgs::Float64 setpoint_msg;
    std_msgs::Float64 state_msg;
    state_msg.data = sensor_z;
    setpoint_msg.data = follow_dist;
    ros::Duration(0.1).sleep();

    while (grab_object != true){
        // Read dist sensor:
        state_msg.data = sensor_z;  // Can use sensor_z_mean instead if using mean filter
        printf("State: %f\n", state_msg.data);

        // Send state and setpoint to PID controller:
        state_pid.publish(state_msg);
        setpoint_pid.publish(setpoint_msg);
        // Robot velocity to follow the object:
        z_robot_vel = output_pid;

        // read current pose of the robot:
        current_pose = move_group_interface_arm.getCurrentPose("ee_link");
        double current_pose_position_x = current_pose.pose.position.x;
        double current_pose_position_y = current_pose.pose.position.y;
        double current_pose_position_z = current_pose.pose.position.z;
        double current_setpoint = setpoint_msg.data;
        // Capture curent time:
        ros::Time current_time = ros::Time::now();
        // Create a structure to store the pose and velocity
        PoseAndVelocity data_point;
        data_point.x = current_pose_position_x;
        data_point.y = current_pose_position_y;
        data_point.z = current_pose_position_z;
        data_point.z_robot_vel = z_robot_vel;
        data_point.setpoint = current_setpoint;
        data_point.state = sensor_z;        
        data_point.time = current_time;
        // Save the data to the vector
        pose_and_velocity_history.push_back(data_point);

        // Publish speedl command to the robot:
        if (state_msg.data != 0 && state_msg.data != 1.5){
            // Only if valid data from the sensor
            speedl_msg.data = "speedl([0,0," + std::to_string(z_robot_vel) + ",0,0,0], 5, 0.015)";
            speedlPublisher.publish(speedl_msg);
        }
        else{
            // If the sensor is not working, stop the robot
            speedl_msg.data = "speedl([0,0,0,0,0,0], 0.5, 0.01)";
            speedlPublisher.publish(speedl_msg);
        }

        // Follow the object for 200 iterations continously (stay within +/- 0.05m of the setpoint follow target):
        if (sensor_z > follow_dist - 0.05 && sensor_z < follow_dist + 0.05){
            counter_follow_object = counter_follow_object + 1;
            if (counter_follow_object > 200){
                // If the object is within the range for 200 iterations, move to grab it
                move_to_grab = true;
                setpoint_msg.data = grab_dist;
                std::cout << "Move to grab object" << std::endl;
            }
        }
        else{
            // Reset counter if the object is not within the range
            counter_follow_object = 0;
        }

        // If robot is approaching to grab check if the target is close enough to grab it:
        if (sensor_z < grab_dist && move_to_grab == true){
            // Update grab_object flag to exit the following loop
            grab_object = true;
            // Stop the robot:
            speedl_msg.data = "speedl([0,0,0,0,0,0], 0.5, 0.1)";
            speedlPublisher.publish(speedl_msg);
            // Disable PID controller:
            std_msgs::Bool enable_msg;
            enable_msg.data = false;
            pid_enable.publish(enable_msg);
            ros::Duration(0.1).sleep();
        }

        // Update command at XX Hz
        ros::Duration(0.01).sleep();
    }
}

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
    // TODO: This causes some problems sometimes?? Is read not always correct?
    current_pose = move_group_interface_arm.getCurrentPose("ee_link");  
    std::cout << current_pose << std::endl; 
    target_pose.position = current_pose.pose.position; 
    target_pose.position.z = current_pose.pose.position.z + 0.2;
    std::cout << target_pose << std::endl;
    move_group_interface_arm.setPoseTarget(target_pose);

    // Plan the trajectory and move to it
    planAndMove();
    ros::Duration(1).sleep();

    // Save the metrics (pose_and_velocity_history) to a csv file here
    saveDataToCSV("/UR10_mbzirc_ws/pid_metrics.csv");
}

void control::saveDataToCSV(const std::string& filename) {
    std::ofstream csv_file(filename);

    if (csv_file.is_open()) {
        // Write header
        csv_file << "X,Y,Z,Z_Robot_Vel,Setpoint,State,Time\n";

        // Write pose and velocity values
        for (const auto& data_point : pose_and_velocity_history) {
            csv_file << data_point.x << "," << data_point.y << "," << data_point.z << ","
                     << data_point.z_robot_vel << "," << data_point.setpoint << ","
                     << data_point.state << "," << data_point.time << "\n";
        }

        csv_file.close();
        std::cout << "Metrics saved to " << filename << std::endl;
    } else {
        std::cerr << "Unable to open the CSV file.\n";
    }
}

void control::goFinalPosition(){
    
    std::cout << "Planning to final position" << std::endl;

    // Set final position coordinates
    orientation.setRPY(-(2* M_PI)/4, 0, 0);
    target_pose.orientation = tf2::toMsg(orientation);
    target_pose.position.x = 0.5;
    target_pose.position.y = 0.5;
    target_pose.position.z = 0.5;
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
        target_pose.position.x = -0.07; 
        target_pose.position.y = 0.55;
        target_pose.position.z =  0.5;

        move_group_interface_arm.setPoseTarget(target_pose);
        planAndMove();
        ros::Duration(5).sleep();

        std::cout << "Planning pos 2" << std::endl;

        orientation.setRPY(-(2* M_PI)/4, 0, 0);
        target_pose.orientation = tf2::toMsg(orientation);
        target_pose.position.x = -0.07; 
        target_pose.position.y = 0.55;
        target_pose.position.z =  0.8;

        // orientation.setRPY(-(2* M_PI)/4, 0, 0);
        // target_pose.orientation = tf2::toMsg(orientation);
        // target_pose.position.x = -0.07; 
        // target_pose.position.y = 0.59;
        // target_pose.position.z =  0.170;

        move_group_interface_arm.setPoseTarget(target_pose);
        planAndMove();
        ros::Duration(5).sleep();
    }

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


void control::runAll(){

    bool object_grabbed = false;

    while (object_grabbed != true){
        // Initial scan to detect target object
        initialScan();

        // Move above object (gets coordinates from camera)
        // Moves towards the object until within a certain distance
        followObject();

        // Grab object and to initial position
        grabObject();

        // Check if the object is grabbed (by checking the distance sensor)
        if (sensor_z < 0.1){
            // If the object is grabbed, set object_grabbed flag to true to exit the loop
            object_grabbed = true;
        }
        else{
            // If the object is not grabbed, detach the box (in moveit) and try again
            move_group_interface_arm.detachObject(target_box_object.id);
        }
    }
    // Move to final position with the box
    goFinalPosition();
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

    if (!nh_.param<bool>("dist_sensor", dist_sensor, true)) {
        ROS_WARN_STREAM("Distance sensor is being used: " << dist_sensor);
    } 
}