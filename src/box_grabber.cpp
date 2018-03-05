//
// Created by y3rs tr00ly and Pete Blacker on 13/12/17.
//
#include <tf2/LinearMath/Transform.h>
#include <ros/ros.h>/* ros/ros.h is a convenience include that
		       includes all the headers necessary to use
		       the most common public pieces of the ROS system.
		       In this code, it is used to create the pose publisher.*/

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf2_ros/transform_listener.h>

#include <robotiq_c_model_control/CModel_robot_output.h>

using namespace std;

ros::Publisher *pose_pub_ptr;
ros::Publisher *debug_pose_pub_ptr;
//ros::Subscriber *pose_sub_ptr;
moveit::planning_interface::MoveGroupInterface *move_group_ptr;
geometry_msgs::PoseStamped lastPose;
geometry_msgs::PoseStamped endEffectorTargetPose;

ros::Publisher *gripperPubPtr;

moveit::planning_interface::MoveGroupInterface::Plan grabPlan;
bool grabPlanned = false;

moveit_visual_tools::MoveItVisualTools *visualToolsPtr;

void setupCollisionObject()
{
    // setup planning scene diffs publisher
    ros::NodeHandle node_handle;
    ros::Publisher planning_scene_diff_publisher = node_handle.advertise<moveit_msgs::PlanningScene>    ("planning_scene", 1);
    while (planning_scene_diff_publisher.getNumSubscribers() < 1)
    {
        ros::WallDuration sleep_t(0.5);
        sleep_t.sleep();
    }

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit_msgs::AttachedCollisionObject collision_object;
    collision_object.object.header.frame_id = "base_link";//move_group_ptr->getPlanningFrame();

    /* The id of the object is used to identify it. */
    collision_object.object.id = "table";

    /* Define a box to add to the world. */
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 1.53;
    primitive.dimensions[1] = 0.76;
    primitive.dimensions[2] = 0.92;

    /* A pose for the box (specified relative to frame_id) */
    geometry_msgs::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = 0;
    box_pose.position.y = 0.21;
    box_pose.position.z = 0-(0.92/2);

    collision_object.object.primitives.push_back(primitive);
    collision_object.object.primitive_poses.push_back(box_pose);
    collision_object.object.operation = collision_object.object.ADD;

    moveit_msgs::PlanningScene planningScene;
    planningScene.world.collision_objects.push_back(collision_object.object);
    planningScene.is_diff = true;
    planning_scene_diff_publisher.publish(planningScene);

    //std::vector<moveit_msgs::CollisionObject> collision_objects;
    //collision_objects.push_back(collision_object);

    ROS_INFO("Add an object into the world");
    //planning_scene_interface.addCollisionObjects(collision_objects);

    // Show text in Rviz of status
    //Eigen::Affine3d text_pose;
    //visualToolsPtr->publishText(text_pose, "Add object", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
    //visualToolsPtr->trigger();

   /* Sleep so we have time to see the object in RViz */
   sleep(2.0);
}

void sendGripperMsg(int position, int speed=100, int force=150)
{
    robotiq_c_model_control::CModel_robot_output closeGripperMsg;
    closeGripperMsg.rACT = 1;
    closeGripperMsg.rGTO = 1;
    closeGripperMsg.rATR = 0;
    closeGripperMsg.rPR = max(0, min(255, position));
    closeGripperMsg.rSP = speed;
    closeGripperMsg.rFR = force;
    gripperPubPtr->publish(closeGripperMsg);
}

void planCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& nothing)
{
    ROS_INFO("Preveiwing plan to target");

    try{

        geometry_msgs::PoseStamped target_pose;
        target_pose.header.frame_id="box_grab_frame";
        target_pose.header.stamp=ros::Time::now();
        target_pose.pose.orientation.x = 0;
        target_pose.pose.orientation.y = 0;
        target_pose.pose.orientation.z = 0;
        target_pose.pose.orientation.w = 1;
        target_pose.pose.position.x = 0;
        target_pose.pose.position.y = 0;
        target_pose.pose.position.z = 0;

        move_group_ptr->setMaxVelocityScalingFactor(.025);
        move_group_ptr->setPoseTarget(target_pose);

        move_group_ptr->plan(grabPlan);
        grabPlanned = true;
    }
    catch (tf2::TransformException &ex) {}
}


void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& markerPose)
{
    if(grabPlanned)
    {
        ROS_INFO("Moving...");

        move_group_ptr->execute(grabPlan);

        sendGripperMsg(255);
    }

    //sleep(5);
    //ros::shutdown();
}

int main(int argc, char **argv) {
/*
 * Initialize ROS. This allows ROS to do name remapping through the
 * command line -- not important for now. This is also where we specify 
 * the name of our node. Node names must be unique in a running system.
 * */
    ros::init(argc, argv,"move_group_interface_tutorial");
 /* 
  * Create a handle to this process' node. The first NodeHandle created will
  *  actually do the initialization of the node, and the last one destructed
  *   will cleanup any resources the node was using.
  *   */
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // Setup a Publisher
    // ^^^^^^^^^^^^^^^^^
    ros::Publisher gripperPub = node_handle.advertise<robotiq_c_model_control::CModel_robot_output>("/CModelRobotOutput", 20);
    gripperPubPtr = &gripperPub;

    // Publish the desired pose. This can be added as a topic in rviz.
    //ros::Publisher pose_pub = node_handle.advertise<geometry_msgs::PoseStamped>("desired_pose", 1000);
    //pose_pub_ptr = &pose_pub;

    // Publish debugger pose which is offset by 10cm from the markerPose.
    ros::Publisher debug_pose_pub = node_handle.advertise<geometry_msgs::PoseStamped>("debug_pose", 1000);
    debug_pose_pub_ptr = &debug_pose_pub;
    // Setup 2 Subscribers
    // ^^^^^^^^^^^^^^^^^^^
    // Subscribe to the desired pose i.e. pose of QR code from visp.
    // As opposed to a publisher, a callback function is used.
    //ros::Subscriber pose_sub = node_handle.subscribe("/visp_auto_tracker/object_position", 1000, desPosCallback);

   ros::Subscriber goal_sub = node_handle.subscribe("/goal", 1000, goalCallback);
   ros::Subscriber init_sub = node_handle.subscribe("/initialpose", 1000, planCallback);

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    // Moveit
    moveit::planning_interface::MoveGroupInterface move_group("manipulator");
    //move_group.setPlannerId("RRTConnectkConfigDefault");
    move_group_ptr = &move_group;

    moveit_visual_tools::MoveItVisualTools visualTools("odom_combined");
    visualToolsPtr = &visualTools;

    ROS_INFO("Planning reference frame: %s", move_group.getPlanningFrame().c_str());
    setupCollisionObject();

  // Raw pointers are frequently used to refer to the planning group for improved performance.
    const robot_state::JointModelGroup *joint_model_group = move_group.getCurrentState()->getJointModelGroup("manipulator");

    // Visualization
    // ^^^^^^^^^^^^^
    //
    // The package MoveItVisualTools provides many capabilties for visualizing objects, robots,
    // and trajectories in Rviz as well as debugging tools such as step-by-step introspection of a script
    //namespace rvt = rviz_visual_tools;
    //moveit_visual_tools::MoveItVisualTools visual_tools("odom_combined");
    //visual_tools.deleteAllMarkers();

    // Remote control is an introspection tool that allows users to step through a high level script
    // via buttons and keyboard shortcuts in Rviz
//    visual_tools.loadRemoteControl();

    // Rviz provides many types of markers, in this demo we will use text, cylinders, and spheres
    //Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
    //text_pose.translation().z() = 1.75; // above head of PR2
//    visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);

    // Batch publishing is used to reduce the number of messages being sent to Rviz for large visualizations
//    visual_tools.trigger();

    //geometry_msgs::TransformStamped transformStamped;

    // open gripper
    sendGripperMsg(0);

    ros::Rate r(1); // 1 hz
    while(node_handle.ok())
    {
      r.sleep();
      ros::spinOnce();
    }
   return 0;
}
