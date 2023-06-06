#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/ApplyPlanningScene.h>

#include <visualization_msgs/Marker.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Point.h>

#include <chrono>
#include <numeric>

// Include tf2 for transformation
 #include <tf2_ros/buffer.h>
 #include <tf2_ros/transform_listener.h>
 #include <tf2_geometry_msgs/tf2_geometry_msgs.h>


bool scan_finished; 
tf2_ros::Buffer tf_buffer;
geometry_msgs::Point target_pose_camera_frame;
//frames
const std::string camera_frame = "camera_color_optical_frame";
const std::string base_link_frame = "base_link";

double target_x; 
double target_y; 
double target_z;
geometry_msgs::Point target_pose_robot_frame;

bool target_received = false; 


// Callback to received scan finished flag to start detection
void scanFlagCB(const std_msgs::Bool::ConstPtr &msg){
    scan_finished = msg->data; 
    std::cout << "Scan position reached, start box detection" << std::endl;
    std::cout << scan_finished << std::endl;
}

// Callback to receive target pose from camera estimation and publish a rviz marker with the box position
void targetPoseCallback(const geometry_msgs::Pose::ConstPtr &msg){

    // Read target position in camera frame
    target_pose_camera_frame = msg->position;
    std::cout << "Target camera frame: " << target_pose_camera_frame << std::endl;

    target_x = target_pose_robot_frame.x; 
    target_y = target_pose_robot_frame.y; 
    target_z = target_pose_robot_frame.z; 
    target_received = true;
}

// Function to transform positions between 2 frames
geometry_msgs::Point transform_between_frames(geometry_msgs::Point p, const std::string from_frame, const std::string to_frame) {
    
  geometry_msgs::PoseStamped input_pose_stamped;
  input_pose_stamped.pose.position = p;
  input_pose_stamped.header.frame_id = from_frame;
  input_pose_stamped.header.stamp = ros::Time::now();
  geometry_msgs::PoseStamped output_pose_stamped = tf_buffer.transform(input_pose_stamped, to_frame, ros::Duration(1));
  return output_pose_stamped.pose.position;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "target_updater");
    ros::NodeHandle n;

    tf2_ros::TransformListener tfListener(tf_buffer);

    // Define param to know if real camera or dummy camera is used
    bool real_camera = false; 

    if (!n.param<bool>("manual", real_camera, false)) {
        ROS_WARN_STREAM("Did not found manual name. Setted to a standard value: " << real_camera);
    } 
    
    // Define publisher and subscriber
    ros::Subscriber scanFlagSubscriber;
    ros::Publisher targetPoseSubscriber_;
    
    scanFlagSubscriber = n.subscribe("/mission_status", 1, &scanFlagCB);
    targetPoseSubscriber_ = n.advertise<geometry_msgs::Pose>("/position", 1, &targetPoseCallback);

    ros::Publisher target_pose_pub = n.advertise<geometry_msgs::Pose>("target_pose", 1000);
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

    // Parameters
    double rate_time = 0.2;
    
    // Simulating waves movement
    float movement = 0.0;
    float movement_y = -0.0; 
    float movement_x = 0.0;
    bool collision_movement = false;
    

    // ROS spinning must be running for the MoveGroupInterface to get information about the robot's state.
    ros::AsyncSpinner spinner(1);
    spinner.start();
    
    // Define move group interface to control the robot
    static const std::string PLANNING_GROUP_ARM = "ur10_arm";
    moveit::planning_interface::MoveGroupInterface move_group_interface_arm(PLANNING_GROUP_ARM);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface; 
    
    //////////////////////////////////////////////////////////////////
    // ADDING COLLISION OBJECTS TO AVOID WHEN GENERATING TRAJECTORY //
    //////////////////////////////////////////////////////////////////

    // Define collision object to be added to the scenario
    moveit_msgs::CollisionObject collision_object;
    std::vector<moveit_msgs::CollisionObject> collision_objects;

    // TABLE
    collision_object.header.frame_id = move_group_interface_arm.getPlanningFrame();
    collision_object.id = "table";
    // Define the shape of the object
    std::vector<shape_msgs::SolidPrimitive> primitive(3);
    primitive[0].type = primitive[0].BOX;
    primitive[0].dimensions.resize(3);
    primitive[0].dimensions[0] = 3;
    primitive[0].dimensions[1] = 1.6;
    primitive[0].dimensions[2] = 0.1;
    collision_object.primitives.push_back(primitive[0]);

    // Define the pose of the object
    std::vector<geometry_msgs::Pose> box_pose(3);
    box_pose[0].orientation.w = 1.0;
    box_pose[0].position.x = 0;
    box_pose[0].position.y = 0.7;
    box_pose[0].position.z = -0.2;
    collision_object.primitive_poses.push_back(box_pose[0]);

    // Add the object to the planning scene
    collision_objects.push_back(collision_object);
    collision_object.operation = collision_object.ADD;
    
    collision_object.primitive_poses.clear(); 
    collision_object.primitives.clear(); 

    // BOX
    /* collision_object.id = "box1";

    primitive[1].type = primitive[1].BOX;
    primitive[1].dimensions.resize(3);
    primitive[1].dimensions[0] = 0.2;
    primitive[1].dimensions[1] = 0.4;
    primitive[1].dimensions[2] = 0.2;

    box_pose[1].orientation.w = 1.0;
    box_pose[1].position.x = 0.0;
    box_pose[1].position.y = 0.65;
    box_pose[1].position.z = 1.23 - 1.21;

    collision_object.primitives.push_back(primitive[1]);
    collision_object.primitive_poses.push_back(box_pose[1]);

    collision_objects.push_back(collision_object);

    collision_object.primitive_poses.clear(); 
    collision_object.primitives.clear();  */

    // WALL
    collision_object.id = "wall";

    primitive[2].type = primitive[2].BOX;
    primitive[2].dimensions.resize(3);
    primitive[2].dimensions[0] = 2;
    primitive[2].dimensions[1] = 0.1;
    primitive[2].dimensions[2] = 4;

    box_pose[2].orientation.w = 1.0;
    box_pose[2].position.x = 0.0;
    box_pose[2].position.y = -0.4;
    box_pose[2].position.z = 0;

    collision_object.primitives.push_back(primitive[2]);
    collision_object.primitive_poses.push_back(box_pose[2]);

    collision_objects.push_back(collision_object);

    // Apply collisions to scene
    planning_scene_interface.applyCollisionObjects(collision_objects);

    ///////////////////////////////////////////////////////
    // DEFINE POSITION OF THE TARGET BOX TO BE PICKED UP //
    ///////////////////////////////////////////////////////

    // Define position of the target box to be published in camera frame
    geometry_msgs::Pose target_pose;
    target_pose.orientation.w = 1.0;
    target_pose.position.x = 0.3;
    target_pose.position.y = 0;
    target_pose.position.z = 0.9;
    geometry_msgs::Point target_point = target_pose.position;

    // Get target position in 
    const std::string from_frame = "camera_color_optical_frame";
    const std::string to_frame = "base_link";

    geometry_msgs::Point target_robot_frame;
    target_robot_frame = transform_between_frames(target_point, from_frame, to_frame);

    //////////////////////////////////////////////////////////////
    // ADD VISUALIZATION MARKER TO KNOW TARGET POSITION IN RVIZ //
    //////////////////////////////////////////////////////////////
    uint32_t shape = visualization_msgs::Marker::CUBE;
    visualization_msgs::Marker marker;

    marker.header.frame_id = move_group_interface_arm.getPlanningFrame();

    marker.id = 0;
    marker.type = shape;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = 0.3;
    marker.pose.position.y = 0.5;

    marker.pose.orientation.x = 0; 
    marker.pose.orientation.y = 0; 
    marker.pose.orientation.z = 0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;

    marker.color.r = 0.0f; 
    marker.color.g = 0.0f; 
    marker.color.b = 1.0f; 
    marker.color.a = 1.0f; 

    // Rate simulating camera rate
    ros::Rate rate(rate_time);

    marker.pose.position = target_robot_frame;


    int c = 0; 
    while (ros::ok()){
        c += 1;

        // Publish rviz marker from camera detection callback  
        if (target_received == true){
            // Obtain target position in robot frame
            target_pose_robot_frame = transform_between_frames(target_pose_camera_frame, camera_frame, base_link_frame);
            std::cout << "Target robot frame: " << target_pose_robot_frame << std::endl;
            marker.pose.position = target_pose_robot_frame;
            marker_pub.publish(marker);
        }

        // Publish target position and visualization marker if real camera not connected
        if (scan_finished == true && real_camera == false){
            // Simulate target box movement due to waves
            /*target_pose.position.x += movement_x;
            target_pose.position.y += movement_y;
            target_pose.position.z += movement;*/
            target_pose.position.x = target_point.x;
            target_pose.position.y = target_point.y;
            target_pose.position.z = target_point.z;
            std::cout << " Camera publishing target_pose " <<std::endl;
            target_pose_pub.publish(target_pose); 
            
            marker.header.stamp = ros::Time::now();
            /*marker.pose.position.x = target_pose.position.x;
            marker.pose.position.y = target_pose.position.y;
            marker.pose.position.z = target_pose.position.z - 0.05;*/
            target_robot_frame = transform_between_frames(target_point, from_frame, to_frame);
            marker.pose.position = target_robot_frame;
            marker_pub.publish(marker);
        }

        /* movement = -movement;
        if (c%2 == 0)
            movement_x = -movement_x; 
        if (c%3 == 0)
            movement_y = -movement_y;  */

        rate.sleep(); 
        ros::spinOnce();
    }

    ros::shutdown();
    return 0;
}
