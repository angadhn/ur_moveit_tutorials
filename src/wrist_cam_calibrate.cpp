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
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_listener.h>

using namespace std;

ros::Publisher *debug_pose_pub_ptr;
tf::TransformListener *listener_ptr;

double sX=0,sY=0,sZ=0,sRo=0,sPi=0,sYa=0;
int sampleCount=0;

void desPosCallback(const geometry_msgs::PoseStamped::ConstPtr& markerPose)
{
    //debug_pose_pub_ptr->publish(markerPose);

    tf2::Transform camPosition;

    tf2::Transform markerPose2, qrPose;
    markerPose2.setOrigin(tf2::Vector3(0.075,
				       0.2,
				       0.0));
    markerPose2.setRotation(tf2::Quaternion(tf2::Vector3(0,0,1),1.5707));

    qrPose.setOrigin(tf2::Vector3(markerPose->pose.position.x,
				      markerPose->pose.position.y,
				      markerPose->pose.position.z));
    qrPose.setRotation(tf2::Quaternion(markerPose->pose.orientation.x,
			   markerPose->pose.orientation.y,
			   markerPose->pose.orientation.z,
			   markerPose->pose.orientation.w));


	camPosition = markerPose2 * qrPose.inverse();

    markerPose2 = camPosition;

    geometry_msgs::PoseStamped endEffectorTargetPose;
    endEffectorTargetPose.header.frame_id="base_link";
    endEffectorTargetPose.header.stamp=ros::Time(0); // <-- most recent time possible
    endEffectorTargetPose.pose.orientation.x = markerPose2.getRotation()[0];
    endEffectorTargetPose.pose.orientation.y = markerPose2.getRotation()[1];
    endEffectorTargetPose.pose.orientation.z = markerPose2.getRotation()[2];
    endEffectorTargetPose.pose.orientation.w = markerPose2.getRotation()[3];
    endEffectorTargetPose.pose.position.x = markerPose2.getOrigin()[0];
    endEffectorTargetPose.pose.position.y = markerPose2.getOrigin()[1];
    endEffectorTargetPose.pose.position.z = markerPose2.getOrigin()[2];

    try{
	    // transform found camera pose into end effector frame (ee_link)
	    listener_ptr->transformPose("ee_link", endEffectorTargetPose, endEffectorTargetPose);

	    // convert to xyz and rpy and echo to the terminal

	    double x = endEffectorTargetPose.pose.position.x;
	    double y = endEffectorTargetPose.pose.position.y;
	    double z = endEffectorTargetPose.pose.position.z;

	    double roll, pitch, yaw;
	    tf2::Matrix3x3(tf2::Quaternion(endEffectorTargetPose.pose.orientation.x,
				          endEffectorTargetPose.pose.orientation.y,
					  endEffectorTargetPose.pose.orientation.z,
					  endEffectorTargetPose.pose.orientation.w)).getRPY(roll, pitch, yaw);

	    ROS_INFO("New camera link parameters are xyz(%.8f %.8f %.8f) ypr(%.8f %.8f %.8f)",
		     x,y,z,
		     yaw, pitch, roll);

	    sX += x;
	    sY += y;
	    sZ += z;
	    sRo += roll;
	    sPi += pitch;
	    sYa += yaw;
	    ++sampleCount;

	    //ROS_INFO("New camera link parameters are xyz(%.8f %.8f %.8f) rpy(%.8f %.8f %.8f)",
	//	     sX/sampleCount,sY/sampleCount,sZ/sampleCount,
		//     sRo/sampleCount, sPi/sampleCount, sYa/sampleCount);

	    debug_pose_pub_ptr->publish(endEffectorTargetPose);
    }
    catch( tf::TransformException ex)
    {
      ROS_ERROR("transfrom exception : %s",ex.what());
    }
}



int main(int argc, char **argv)
{
    ros::init(argc, argv,"move_group_interface_tutorial");

    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    tf::TransformListener listener;
    listener_ptr = &listener;

    // Publish debugger pose which is offset by 10cm from the markerPose.
    ros::Publisher debug_pose_pub = node_handle.advertise<geometry_msgs::PoseStamped>("debug_pose", 1000);
    debug_pose_pub_ptr = &debug_pose_pub;
    // Setup 2 Subscribers
    // ^^^^^^^^^^^^^^^^^^^
    // Subscribe to the desired pose i.e. pose of QR code from visp.
    // As opposed to a publisher, a callback function is used.
    ros::Subscriber pose_sub = node_handle.subscribe("/visp_auto_tracker/object_position", 1000, desPosCallback);

   
    
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
    ros::Rate r(1); // 1 hz
    while (ros::ok())
    {
//     moveArm();
      r.sleep();
      ros::spinOnce();
    }
   return 0;
}
