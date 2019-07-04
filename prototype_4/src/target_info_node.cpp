#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Header.h"

#include "sensor_msgs/Image.h"
#include <image_transport/image_transport.h>

#include <sensor_msgs/image_encodings.h>
#include <nav_msgs/Odometry.h>

#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/ModelStates.h>

#include "stdlib.h"
#include "stdio.h"
#include <string.h>
#include <cmath>
#include <random_numbers/random_numbers.h>
#include "tf/transform_datatypes.h"
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseArray.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#define FALSE 0
#define TRUE 1
#define TARGET_KEYWORD "Target"
#define SIGMA 0.0
#define MU 0.1
#define NB_TARGETS 3

//----------------------------------------------------------
//                       Global variables
//----------------------------------------------------------


int flag_callback = FALSE;
int flag_callback_drone = FALSE;
int flag_callback_mod = FALSE;
gazebo_msgs::ModelStates gt_ptr;
nav_msgs::Odometry odom_ptr;
std_msgs::Header img_header;




//----------------------------------------------------------
//                        Main
//----------------------------------------------------------

int main(int argc, char **argv)
{
  //----------------------------
  // Init
  ros::init(argc, argv, "dummy_sensor");

  //----------------------------
  // Node handler with image transport
  ros::NodeHandle n;


  //----------------------------
  // Suscribers and publishers
  /*
  message_filters::Subscriber<nav_msgs::Odometry> sub_odom(n, "ground_truth/state", 1);
  message_filters::Subscriber<nav_msgs::Odometry> sub_mod(n, "/gazebo/model_states", 1);
  typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, nav_msgs::Odometry> SyncPolicy;
  message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(10), sub_odom, sub_mod);
  sync.registerCallback(boost::bind(&droneCallback, _1, _2));
  ros::Subscriber sub_mod = n.subscribe("/gazebo/model_states", 1, modelCallback);
  */
  ros::Publisher pub_targets = n.advertise<geometry_msgs::PoseArray>("targets_poses",1000);


  //----------------------------
  // Rate
  ros::Rate loop_rate(30);

  //----------------------------
  // Init msg
  
  // Header
  std_msgs::Header header;
  header.frame_id = "nav";
  header.stamp = ros::Time::now();

  // Init poses
  geometry_msgs::PoseArray targets_poses_msg;
  std::vector<geometry_msgs::Pose> targets_poses;

  geometry_msgs::Pose target_pose_1; geometry_msgs::Pose target_pose_2; geometry_msgs::Pose target_pose_3;

  target_pose_1.position.x = 5.0; target_pose_1.position.y = 0.0; target_pose_1.position.z = 0.5;
  target_pose_1.orientation.x = 0.0; target_pose_1.orientation.y = 0.0; target_pose_1.orientation.z = 0.0; target_pose_1.orientation.w = 0.0;

  target_pose_2.position.x = -2.0; target_pose_2.position.y = -6.0; target_pose_2.position.z = 0.5;
  target_pose_2.orientation.x = 0.0; target_pose_2.orientation.y = 0.0; target_pose_2.orientation.z = 0.0; target_pose_2.orientation.w = 0.0;

  target_pose_3.position.x = 0.0; target_pose_3.position.y = 7.0; target_pose_3.position.z = 0.5;
  target_pose_3.orientation.x = 0.0; target_pose_3.orientation.y = 0.0; target_pose_3.orientation.z = 0.0; target_pose_3.orientation.w = 0.0;

  targets_poses.push_back(target_pose_1);
  targets_poses.push_back(target_pose_2);
  targets_poses.push_back(target_pose_3);

  targets_poses_msg.poses = targets_poses;
  targets_poses_msg.header = header;

  //----------------------------
  // While loop
  while (ros::ok())
  {
    // Publish PoseArray message
    pub_targets.publish(targets_poses_msg);

    ros::spinOnce();

    loop_rate.sleep();
  }
  return 0;
}
