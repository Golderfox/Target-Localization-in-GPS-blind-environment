#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Header.h"


#include <nav_msgs/Odometry.h>

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
#define LIMITED_SENSOR 0

//----------------------------------------------------------
//                       Global variables
//----------------------------------------------------------


int flag_callback = FALSE;
int flag_callback_drone = FALSE;
int flag_callback_mod = FALSE;

nav_msgs::Odometry odom_ptr;
std_msgs::Header img_header;



//----------------------------------------------------------
//                    Convertion function
//----------------------------------------------------------

// --> convert Target pose from world's frame to drone's frame
geometry_msgs::Pose getRelativePose(geometry_msgs::Pose drone_pose, geometry_msgs::Pose target_pose)
{
  geometry_msgs::Pose relative_pose;

  relative_pose.position.x = target_pose.position.x - drone_pose.position.x;
  relative_pose.position.y = target_pose.position.y - drone_pose.position.y;
  relative_pose.position.z = target_pose.position.z - drone_pose.position.z;

  relative_pose.orientation.x = target_pose.orientation.x;
  relative_pose.orientation.y = target_pose.orientation.y;
  relative_pose.orientation.z = target_pose.orientation.z;
  relative_pose.orientation.w = target_pose.orientation.w;

  return relative_pose;
}


//----------------------------------------------------------
//                       DroneCallback
//----------------------------------------------------------
void droneCallback(const nav_msgs::Odometry& msg)
{
  ROS_INFO("Odometry");
  odom_ptr = msg;
  flag_callback_drone = TRUE;
}

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
  ros::Subscriber sub_odom = n.subscribe("ground_truth/state", 1, droneCallback);
  ros::Publisher pub_poses = n.advertise<geometry_msgs::PoseArray>("targets_poses",1000);

  tf::TransformListener listener;
  //----------------------------
  // Rate
  ros::Rate loop_rate(30);

  //----------------------------
  // Init targets
  geometry_msgs::Pose target_pose_1; geometry_msgs::Pose target_pose_2; geometry_msgs::Pose target_pose_3;
  std::vector<geometry_msgs::Pose> targets_poses;
  std::vector<geometry_msgs::Pose> reachable_targets_poses;
  // Target 1
  target_pose_1.position.x = 5.0; target_pose_1.position.y = 0.0; target_pose_1.position.z = 0.5;
  target_pose_1.orientation.x = 0.0; target_pose_1.orientation.y = 0.0; target_pose_1.orientation.z = 0.0; target_pose_1.orientation.w = 0.0;
  targets_poses.push_back(target_pose_1);

  // Target 2
  target_pose_2.position.x = -2.0; target_pose_2.position.y = -6.0; target_pose_2.position.z = 0.5;
  target_pose_2.orientation.x = 0.0; target_pose_2.orientation.y = 0.0; target_pose_2.orientation.z = 0.0; target_pose_2.orientation.w = 0.0;
  targets_poses.push_back(target_pose_2);

  // Target 3
  target_pose_3.position.x = 0.0; target_pose_3.position.y = 7.0; target_pose_3.position.z = 0.5;
  target_pose_3.orientation.x = 0.0; target_pose_3.orientation.y = 0.0; target_pose_3.orientation.z = 0.0; target_pose_3.orientation.w = 0.0;
  targets_poses.push_back(target_pose_3);

  geometry_msgs::Pose drone_pose;
  geometry_msgs::Pose target_relative_pose_1; geometry_msgs::Pose target_relative_pose_2; geometry_msgs::Pose target_relative_pose_3;
  geometry_msgs::PoseArray targets_detected_msg;

  
  random_numbers::RandomNumberGenerator generator;

  //----------------------------
  // While loop
  while (ros::ok())
  {
    int reacheable_target_counter = 0;
    // frame link front cam 0.21 0.0 0.01
    reachable_targets_poses.clear();
    std_msgs::Header header;
    tf::StampedTransform transform;

    // Header
    header.frame_id = "1";
    header.stamp = ros::Time::now();

    if(LIMITED_SENSOR && flag_callback_drone){
      // frame link front cam 0.21 0.0 0.01
      tf::Transform world_to_cam;

      drone_pose = odom_ptr.pose.pose;
      drone_pose.position.x += 0.21;
      drone_pose.position.y += 0.0;
      drone_pose.position.z += 0.01;

      tf::poseMsgToTF(drone_pose, world_to_cam);
      for(int i=0; i<targets_poses.size(); i++){
        geometry_msgs::Pose cam_target_pose;
        tf::Transform world_to_target;
        tf::poseMsgToTF(targets_poses[i], world_to_target);
        tf::Transform cam_to_target = world_to_cam.inverse()*world_to_target;
        tf::poseTFToMsg(cam_to_target, cam_target_pose);

        if(cam_target_pose.position.x >= 0.0){
          
          reachable_targets_poses.push_back(targets_poses[i]);

          //----------------------------
          // Add gaussian noise
          reachable_targets_poses[reacheable_target_counter].position.x += generator.gaussian(SIGMA,MU);
          reachable_targets_poses[reacheable_target_counter].position.y += generator.gaussian(SIGMA,MU);
          reachable_targets_poses[reacheable_target_counter].position.z += generator.gaussian(SIGMA,MU);
          targets_detected_msg.poses = reachable_targets_poses;
          targets_detected_msg.header = odom_ptr.header;

          reacheable_target_counter += 1;
          
          pub_poses.publish(targets_detected_msg);
        }
      }
    }
    else if(!LIMITED_SENSOR && flag_callback_drone){

      for(int i=0; i<targets_poses.size(); i++){
        reachable_targets_poses.push_back(targets_poses[i]);
        //----------------------------
        // Add gaussian noise
        reachable_targets_poses[i].position.x += generator.gaussian(SIGMA,MU);
        reachable_targets_poses[i].position.y += generator.gaussian(SIGMA,MU);
        reachable_targets_poses[i].position.z += generator.gaussian(SIGMA,MU);
        targets_detected_msg.poses = reachable_targets_poses;
        targets_detected_msg.header = odom_ptr.header;

        pub_poses.publish(targets_detected_msg);
      }

      /*
      // Publish pose if target is in front of the drone
      if(target_relative_pose.position.x > 0){
        target_msg.pose = target_relative_pose;
        target_msg.header = odom_ptr.header;
        pub_pose.publish(target_msg);
      }
      */
    }
    ros::spinOnce();

    loop_rate.sleep();
  }
  return 0;
}
