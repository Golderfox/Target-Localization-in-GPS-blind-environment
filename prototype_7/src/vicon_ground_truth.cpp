#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Header.h"

#include <nav_msgs/Odometry.h>


#include "stdlib.h"
#include "stdio.h"
#include <string.h>
#include <cmath>
#include "tf/transform_datatypes.h"

#include <prototype_3/DKF.h>
#include <prototype_3/DKFStamped.h>
#include <prototype_3/DKF_multi.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#define FALSE 0
#define TRUE 1
#define COM_RANGE 0.5

//----------------------------------------------------------
//                       Global variables
//----------------------------------------------------------


int flag_callback = FALSE;
int flag_callback_drone = FALSE;
int flag_callback_mod = FALSE;

geometry_msgs::Pose drone_pose_1;
geometry_msgs::Pose drone_pose_2;

geometry_msgs::Pose target_pose;


std_msgs::Header header;


//----------------------------------------------------------
//                    Defining functions
//----------------------------------------------------------

// --> return the distance between two drones
float getDistance(geometry_msgs::Pose drone_pose1, geometry_msgs::Pose drone_pose2)
{
    sqrt(std::pow((drone_pose1.position.x)-(drone_pose2.position.x),2.0) + std::pow((drone_pose1.position.y)-(drone_pose2.position.y),2.0) + std::pow((drone_pose1.position.z)-(drone_pose2.position.z),2.0));
}

// --> return the index list of neighbours for a given drone
std::vector<int> getNeighbours(std::vector<geometry_msgs::Pose> drone_poses, int drone_index)
{
  std::vector<int> Neighbours_list;
  for(int i=0; i<drone_poses.size(); i++){
    if(i != drone_index){
      float distance = getDistance(drone_poses[i], drone_poses[drone_index]);
      if(distance < COM_RANGE){
        Neighbours_list.push_back(i);
      }
    }
  }
  return Neighbours_list;
}



//----------------------------------------------------------
//                       DroneCallback
//----------------------------------------------------------


void viconCallback(const geometry_msgs::TransformStamped::ConstPtr drone_msg_1, const geometry_msgs::TransformStamped::ConstPtr drone_msg_2, const geometry_msgs::TransformStamped::ConstPtr target_msg){
  ROS_INFO("Odometry");
  header = drone_msg_1->header;

  drone_pose_1.position.x = drone_msg_1->transform.translation.x;
  drone_pose_1.position.y = drone_msg_1->transform.translation.y;
  drone_pose_1.position.z = drone_msg_1->transform.translation.z;
  drone_pose_1.orientation = drone_msg_1->transform.rotation;

  drone_pose_2.position.x = drone_msg_2->transform.translation.x;
  drone_pose_2.position.y = drone_msg_2->transform.translation.y;
  drone_pose_2.position.z = drone_msg_2->transform.translation.z;
  drone_pose_2.orientation = drone_msg_2->transform.rotation;

  target_pose.position.x = target_msg->transform.translation.x;
  target_pose.position.y = target_msg->transform.translation.y;
  target_pose.position.z = target_msg->transform.translation.z;
  target_pose.orientation = target_msg->transform.rotation;

  flag_callback_drone = TRUE;
}


//----------------------------------------------------------
//                        Main
//----------------------------------------------------------

int main(int argc, char **argv){
  //----------------------------
  // Init
  ros::init(argc, argv, "vicon_ground_truth");

  //----------------------------
  // Node handler with image transport
  ros::NodeHandle n;

  //----------------------------
  // Suscribers and publishers

  // DKF subscribers
  message_filters::Subscriber<geometry_msgs::TransformStamped> sub_vicon1(n, "/vicon/quadrotor1/quadrotor1", 1);
  message_filters::Subscriber<geometry_msgs::TransformStamped> sub_vicon2(n, "/vicon/quadrotor0/quadrotor0", 1);
  message_filters::Subscriber<geometry_msgs::TransformStamped> sub_vicon_target(n, "/vicon/tag/tag", 1);



  // Synchroniser policy
  typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::TransformStamped, geometry_msgs::TransformStamped, geometry_msgs::TransformStamped> SyncPolicy;
  message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(10), sub_vicon1, sub_vicon2, sub_vicon_target);
  sync.registerCallback(boost::bind(&viconCallback, _1, _2, _3));

  // Neighbours message publishers
  ros::Publisher pub_odom1 = n.advertise<nav_msgs::Odometry>("/quadrotor1/ground_truth/state",1000);
  ros::Publisher pub_odom2 = n.advertise<nav_msgs::Odometry>("/quadrotor0/ground_truth/state",1000);
  ros::Publisher pub_target = n.advertise<geometry_msgs::PoseStamped>("/target",1000);

  //----------------------------
  // Init messages

  nav_msgs::Odometry drone_msg_1;
  nav_msgs::Odometry drone_msg_2;
  geometry_msgs::PoseStamped target_msg;

  //----------------------------
  // Rate
  ros::Rate loop_rate(30);

  //----------------------------
  // While loop
  while (ros::ok()){

    //----------------------------
    // If callback is received
    if (flag_callback_drone == TRUE){

      // Assign message of drone 1
      drone_msg_1.header = header;
      drone_msg_1.pose.pose = drone_pose_1;

      // Assign message of drone 2
      drone_msg_2.header = header;
      drone_msg_2.pose.pose = drone_pose_2;

      // Assign message of target
      target_msg.header = header;
      target_msg.pose = target_pose;

      // Publish messages to drones od the swarm
      pub_odom1.publish(drone_msg_1);
      pub_odom2.publish(drone_msg_2);
      pub_target.publish(target_msg);
    }
  
    ros::spinOnce();

    loop_rate.sleep();
  }
  return 0;
}