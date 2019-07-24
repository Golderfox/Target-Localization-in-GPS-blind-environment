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
/*
void drone1Callback(const nav_msgs::Odometry& msg)
{
  ROS_INFO("Odometry");
  odom_ptr1 = msg;
  flag_callback_drone = TRUE;
}

void drone2Callback(const nav_msgs::Odometry& msg)
{
  ROS_INFO("Odometry");
  odom_ptr2 = msg;
  flag_callback_drone = TRUE;
}

void drone3Callback(const nav_msgs::Odometry& msg)
{
  ROS_INFO("Odometry");
  odom_ptr3 = msg;
  flag_callback_drone = TRUE;
}
*/



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
  ros::Publisher pub_target = n.advertise<geometry_msgs::PoseStamped>("/target",1000);

  //----------------------------
  // Init messages
  geometry_msgs::PoseStamped target_msg;
  geometry_msgs::Pose target_pose;
  target_pose.position.x = 0.0;
  target_pose.position.y = 0.0;
  target_pose.position.z = 0.5;
  target_pose.orientation.x = 0.0;
  target_pose.orientation.y = 0.0;
  target_pose.orientation.z = 0.0;
  target_pose.orientation.w = 0.0;

  //----------------------------
  // Rate
  ros::Rate loop_rate(30);

  //----------------------------
  // While loop
  while (ros::ok()){


    // Header
    header.frame_id = "nav";
    header.stamp = ros::Time::now();

    // Assign message of target
    target_msg.header = header;
    target_msg.pose = target_pose;

    // Publish messages to drones od the swarm
    pub_target.publish(target_msg);
    
  
    ros::spinOnce();

    loop_rate.sleep();
  }
  return 0;
}