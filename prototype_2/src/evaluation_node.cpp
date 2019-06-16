#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Header.h"

#include "sensor_msgs/Image.h"
//#include <image_transport/image_transport.h>
//#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <nav_msgs/Odometry.h>

#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/ModelStates.h>

//#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/highgui/highgui.hpp>
#include "stdlib.h"
#include "stdio.h"
#include <string.h>
#include <cmath>
#include "tf/transform_datatypes.h"
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/PointCloud.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <tf/transform_listener.h>

#define FALSE 0
#define TRUE 1
#define TARGET_KEYWORD "Target"
#define CAMERA_OFFSET_X 0.21
#define CAMERA_OFFSET_Y 0.0
#define CAMERA_OFFSET_Z 0.01

//----------------------------------------------------------
//                       Global variables
//----------------------------------------------------------

//cv_bridge::CvImagePtr cv_ptr;

// Flags used to detect whether suscribers have received any message
int flag_callback = FALSE;
int flag_callback_drone = FALSE;
int flag_callback_mod = FALSE;

// Gazebo Model states msg
gazebo_msgs::ModelStates gt_ptr;

// Odometry msg
nav_msgs::Odometry odom_ptr;
geometry_msgs::Pose estimate;
geometry_msgs::Pose odom_pose;

//tf::TransformListener listener_;

//std_msgs::Header img_header;


//----------------------------------------------------------
//                    Distance function
//----------------------------------------------------------

// --> get the distance between two poses
float getDistance(geometry_msgs::Pose pose1, geometry_msgs::Pose pose2){
  float dist = sqrt(std::pow((pose1.position.x-pose2.position.x),2.0) + std::pow((pose1.position.y-pose2.position.y),2.0) + std::pow((pose1.position.z-pose2.position.z),2.0));
  ROS_INFO("dist: %f", dist);
  return dist;
}

// --> get the relative pose 
geometry_msgs::Pose getRelativePose(geometry_msgs::Pose drone_pose, geometry_msgs::Pose target_abs_pose){
    geometry_msgs::Pose rel_pose;
    rel_pose.position.x = target_abs_pose .position.x - drone_pose.position.x;
    rel_pose.position.y = target_abs_pose .position.y - drone_pose.position.y;
    rel_pose.position.z = target_abs_pose .position.z - drone_pose.position.z;

    rel_pose.orientation.x = drone_pose.orientation.x;
    rel_pose.orientation.y = drone_pose.orientation.y;
    rel_pose.orientation.z = drone_pose.orientation.z;
    rel_pose.orientation.w = drone_pose.orientation.w;
    
    return rel_pose;
}


//----------------------------------------------------------
//                       Callback
//----------------------------------------------------------

void Callback(const geometry_msgs::PoseStamped::ConstPtr msgEstimate) //, const nav_msgs::Odometry::ConstPtr msgOdom)
{
  // Set flag
  flag_callback = TRUE;



  //odom_pose = msgOdom->pose.pose;
  estimate = msgEstimate->pose;

  /*
  odom_pose.position.x += CAMERA_OFFSET_X;
  odom_pose.position.y += CAMERA_OFFSET_Y;
  odom_pose.position.z += CAMERA_OFFSET_Z;
  */
  
  

}
//----------------------------------------------------------
//                        Main
//----------------------------------------------------------

int main(int argc, char **argv)
{
  //----------------------------
  // Init
  ros::init(argc, argv, "eval_node");

  //----------------------------
  // Node handler with image transport
  ros::NodeHandle n;
  //image_transport::ImageTransport it(n);

  //----------------------------
  // Suscribers and publishers
  //ros::Subscriber sub_mod = n.subscribe("/gazebo/model_states", 1, modelCallback);
  //message_filters::Subscriber<geometry_msgs::PoseStamped> sub_estimate(n, "/aruco_single/pose", 1);
  //message_filters::Subscriber<nav_msgs::Odometry> sub_odom(n, "/quadrotor1/ground_truth/state", 1);
  //typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseStamped, nav_msgs::Odometry> MySyncPolicy;

  ros::Subscriber sub_estimate = n.subscribe("/aruco_single/pose", 1, Callback);
  // Message synchronizer
  //message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sub_estimate, sub_odom);
  //sync.registerCallback(boost::bind(&Callback, _1, _2));
  ros::Publisher pub_distance = n.advertise<std_msgs::Float32>("distance",1000);

 

  //----------------------------
  // Target's pose
    geometry_msgs::Pose Target_abs_pose;
    geometry_msgs::Pose Estimate_abs_pose;
    Target_abs_pose.position.x = 9.5; Target_abs_pose.position.y = 0.0; Target_abs_pose.position.z = 0.5;
    Target_abs_pose.orientation.x = 0.0; Target_abs_pose.orientation.y = 0.0; Target_abs_pose.orientation.z = 0.0; Target_abs_pose.orientation.w = 0.0;

  //----------------------------
  // Rate
  ros::Rate loop_rate(30);

  //----------------------------
  // While loop
  while (ros::ok()){
    // frame link front cam 0.21 0.0 0.01

    std_msgs::Header header;

    std_msgs::Float32 msg_error;
    tf::Transform world_to_cam;
    tf::Transform cam_to_target;

    // Header
    header.frame_id = "1";
    header.stamp = ros::Time::now();

    if(flag_callback == TRUE){


      // Set drone matrix transform
      /*
      cv::Mat drone_transform = getExtrinsicMatrix(drone_pose);
      cv::Mat drone_projection =  Intrisic * drone_transform;
      */
     /*
      tf::poseMsgToTF(odom_pose, world_to_cam);
      tf::poseMsgToTF(estimate, cam_to_target);
      tf::Transform world_to_target = world_to_cam * cam_to_target;
      tf::poseTFToMsg(world_to_target, Estimate_abs_pose);
      */
      //geometry_msgs::Pose rel_pose = getRelativePose(odom_pose, Target_abs_pose);
      float error = getDistance(estimate, Target_abs_pose);

      // Completing bounding box message
      msg_error.data = error;

    }
    
    // Publishing message
    pub_distance.publish(msg_error);

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;

}
