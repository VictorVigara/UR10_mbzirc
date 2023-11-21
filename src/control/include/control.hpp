#ifndef __CONTROL_H__
#define __CONTROL_H__

#include <ros/ros.h>
#include <memory>

// MoveitCpp
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

// ROS
#include <geometry_msgs/PointStamped.h>
#include "geometry_msgs/Point.h"
#include <sensor_msgs/Range.h>
#include <std_msgs/Float64.h>

#include <chrono>


class control{
public:
    control(ros::NodeHandle& nodehandle);

    void subscribeToTopics();
    void createPublishers();
    void loadParameters(); 
    geometry_msgs::Point transform_between_frames(geometry_msgs::Point, const std::string, const std::string);
    void initialScan(); 
    void planAndMove(); 
    void approachTarget(); 
    void grabObject(); 
    void goFinalPosition(); 
    void runAll();
    void attachBox();
    void testSequence();
    void followObject();

    std::string target_pose_topic;
    std::string scan_finished_topic; 


private:
    
    ros::NodeHandle nh_; // we will need this, to pass between "main" and constructor
    
    // Subscribers
    ros::Subscriber targetPoseSubscriber_;
    ros::Subscriber distsensorsubscriber_;
    ros::Subscriber control_effort_subscriber;
    
    //Publishers
    ros::Publisher scanFinishedPublisher;
    ros::Publisher speedlPublisher;
    ros::Publisher setpoint_pid;
    ros::Publisher state_pid;
    ros::Publisher pid_enable;
    
    // Callbacks
    void targetPoseCallback(const geometry_msgs::Pose::ConstPtr &msg);
    void distSensorCallback(const sensor_msgs::Range& range_msg);
    void controlEffortCallback(const std_msgs::Float64& control_effort_msg);
    
    // transforms
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tfListener;

    //frames
    const std::string camera_frame = "camera_color_optical_frame";
    const std::string base_link_frame = "base_link";


    //  Variables // 
    bool manual = true;
    bool dist_sensor;
    int static_scan_time;
    double vel_scaling_factor; 
    double acc_scaling_factor;

    // Moveit
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface; 
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    moveit_msgs::OrientationConstraint ocm;
    moveit::planning_interface::MoveGroupInterface move_group_interface_arm;
    moveit_msgs::CollisionObject target_box_object;
    shape_msgs::SolidPrimitive box_primitive;
    geometry_msgs::Pose grab_pose;

    bool success;
    geometry_msgs::PoseStamped current_pose;
    moveit_msgs::Constraints test_constraints;

    geometry_msgs::Pose target_pose;
    geometry_msgs::Point target_pose_camera_frame;
    geometry_msgs::Point target_pose_robot_frame;

    bool target_received; 
    bool grab_object;
    bool scan_finished;

    bool mission_finished; 

    double target_x; 
    double target_y; 
    double target_z;

    double sensor_z;
    double sensor_z_mean;
    double sensor_z_velocity;
    std::list<double> z_distance_list;

    // PID:
    float output_pid;



    tf2::Quaternion orientation;

    const double tau = 2 * M_PI;
};

#endif // __CONTROL_H__
