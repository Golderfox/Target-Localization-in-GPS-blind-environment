#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Header.h"

#include <nav_msgs/Odometry.h>

#include "iostream"
#include "stdlib.h"
#include "stdio.h"
#include <string.h>
#include <cmath>
#include "tf/transform_datatypes.h"
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <prototype_6/DKF.h>
#include <prototype_6/DKFStamped.h>
#include <prototype_6/DKF_multi.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#define FALSE 0
#define TRUE 1
#define COM_RANGE 0.5
#define TARGET_RADIUS 1.5
#define DRONE_NUMBER 6


//----------------------------------------------------------
//                    Defining global variables
//----------------------------------------------------------

int flag_callback = FALSE;

geometry_msgs::Pose estimate_pose_1;
geometry_msgs::Pose estimate_pose_2;
geometry_msgs::Pose estimate_pose_3;
geometry_msgs::Pose estimate_pose_4;
geometry_msgs::Pose estimate_pose_5;
geometry_msgs::Pose estimate_pose_6;

std::vector<geometry_msgs::Pose> estimates_poses;


std_msgs::Header nav_header;


//----------------------------------------------------------
//                    Defining functions
//----------------------------------------------------------

// --> return the distance between two drones
float getDistance(geometry_msgs::Pose drone_pose1, geometry_msgs::Pose drone_pose2)
{
  return sqrt(std::pow((drone_pose1.position.x)-(drone_pose2.position.x),2.0) + std::pow((drone_pose1.position.y)-(drone_pose2.position.y),2.0) + std::pow((drone_pose1.position.z)-(drone_pose2.position.z),2.0));
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

void droneCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr drone_msg_1, const geometry_msgs::PoseWithCovarianceStamped::ConstPtr drone_msg_2, const geometry_msgs::PoseWithCovarianceStamped::ConstPtr drone_msg_3, const geometry_msgs::PoseWithCovarianceStamped::ConstPtr drone_msg_4, const geometry_msgs::PoseWithCovarianceStamped::ConstPtr drone_msg_5, const geometry_msgs::PoseWithCovarianceStamped::ConstPtr drone_msg_6){
  ROS_INFO("Odometry");
  nav_header = drone_msg_1->header;

  estimate_pose_1 = drone_msg_1->pose.pose;
  
  estimate_pose_2 = drone_msg_2->pose.pose;
  estimate_pose_3 = drone_msg_3->pose.pose;
  estimate_pose_4 = drone_msg_4->pose.pose;
  estimate_pose_5 = drone_msg_5->pose.pose;
  estimate_pose_6 = drone_msg_6->pose.pose;
  estimates_poses.clear();
  estimates_poses.push_back(drone_msg_1->pose.pose);
  estimates_poses.push_back(drone_msg_2->pose.pose);
  estimates_poses.push_back(drone_msg_3->pose.pose);
  estimates_poses.push_back(drone_msg_4->pose.pose);
  estimates_poses.push_back(drone_msg_5->pose.pose);
  estimates_poses.push_back(drone_msg_6->pose.pose);
  flag_callback = TRUE;
}


//----------------------------------------------------------
//                        Main
//----------------------------------------------------------

int main(int argc, char **argv){
  //----------------------------
  // Init
  std::cout << "init" <<std::endl;
  ros::init(argc, argv, "mapping_node");
  //----------------------------
  // Node handler with image transport
  ros::NodeHandle n;

  //----------------------------
  // Suscribers and publishers

  // DKF subscribers
  message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped> sub_est1(n, "/quadrotor1/estimate", 1);
  message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped> sub_est2(n, "/quadrotor2/estimate", 1);
  message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped> sub_est3(n, "/quadrotor3/estimate", 1);
  message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped> sub_est4(n, "/quadrotor4/estimate", 1);
  message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped> sub_est5(n, "/quadrotor5/estimate", 1);
  message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped> sub_est6(n, "/quadrotor6/estimate", 1);

  // Synchroniser policy
  typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseWithCovarianceStamped, geometry_msgs::PoseWithCovarianceStamped, geometry_msgs::PoseWithCovarianceStamped, geometry_msgs::PoseWithCovarianceStamped, geometry_msgs::PoseWithCovarianceStamped, geometry_msgs::PoseWithCovarianceStamped> SyncPolicy;
  message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(10), sub_est1, sub_est2, sub_est3, sub_est4, sub_est5, sub_est6);
  sync.registerCallback(boost::bind(&droneCallback, _1, _2, _3, _4, _5, _6));

  /*
  // Neighbours message subscribers
  ros::Subscriber sub_input1 = n.subscribe("/quadrotor1/DKF_out", 1, drone1Callback);
  ros::Subscriber sub_input2 = n.subscribe("/quadrotor2/DKF_out", 1, drone2Callback);
  ros::Subscriber sub_input3 = n.subscribe("/quadrotor3/DKF_out", 1, drone3Callback);
  */

  // Neighbours message publishers
  ros::Publisher pub_markers = n.advertise<visualization_msgs::MarkerArray>("/targets_map",1000);

  //----------------------------
  // Init messages
  visualization_msgs::MarkerArray targets_map_msg;

  std::vector<visualization_msgs::Marker> targets_markers_array;
  std::vector<geometry_msgs::Pose> targets_poses_array;

  //----------------------------
  // Rate
  ros::Rate loop_rate(30);

  //----------------------------
  // While loop
  while (ros::ok()){

    

    //----------------------------
    // If callback is received
    if (flag_callback == TRUE){
      // Clear DKF arrays
      targets_markers_array.clear();
      targets_poses_array.clear();
      std::cout << "callback" <<std::endl;

      if(estimates_poses.size()>0){
        targets_poses_array.push_back(estimates_poses[0]);
      }

      for(int i=1; i<estimates_poses.size(); i++){
        for(int j=0; j<targets_poses_array.size(); j++){
          if(getDistance(estimates_poses[i], targets_poses_array[j])> TARGET_RADIUS){
            targets_poses_array.push_back(estimates_poses[i]);
          }
        }
      }
      targets_markers_array.clear();
      for(int i=0; i<targets_poses_array.size(); i++){
        visualization_msgs::Marker target_marker;
        target_marker.header = nav_header;
        //marker.ns = "my_namespace";
        target_marker.id = i;
        target_marker.type = visualization_msgs::Marker::SPHERE;
        target_marker.action = visualization_msgs::Marker::ADD;
        target_marker.pose = targets_poses_array[i];
        target_marker.scale.x = 1;
        target_marker.scale.y = 1;
        target_marker.scale.z = 1;
        target_marker.color.a = 0.8; // Don't forget to set the alpha!
        target_marker.color.r = 0.0;
        target_marker.color.g = 1.0;
        target_marker.color.b = 0.0;

        targets_markers_array.push_back(target_marker);
      }
      ROS_INFO("pose %d", targets_poses_array.size());
      ROS_INFO("array %d", targets_markers_array.size());
      targets_map_msg.markers = targets_markers_array;
      pub_markers.publish(targets_map_msg);

    }
  
    ros::spinOnce();

    loop_rate.sleep();
  }
  return 0;
}