#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
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

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/Pose.h>

#define FALSE 0
#define TRUE 1
#define COM_RANGE 1.5

//----------------------------------------------------------
//                       Global variables
//----------------------------------------------------------


int flag_callback_estimation = FALSE;
int flag_callback_innovation = FALSE;
int flag_callback_odom = FALSE;
int flag_callback_mod = FALSE;

geometry_msgs::PoseWithCovarianceStamped est_drone1;
geometry_msgs::PoseWithCovarianceStamped est_drone2;
geometry_msgs::PoseWithCovarianceStamped est_drone3;

geometry_msgs::PoseWithCovarianceStamped innov_drone;

nav_msgs::Odometry odom_ptr1;
nav_msgs::Odometry odom_ptr2;
nav_msgs::Odometry odom_ptr3;


//----------------------------------------------------------
//                    Defining functions
//----------------------------------------------------------

// --> return the distance between two drones
double getDistance(geometry_msgs::Pose drone_pose1, geometry_msgs::Pose drone_pose2){
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

void EstimateCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr est_msg_1, const geometry_msgs::PoseWithCovarianceStamped::ConstPtr est_msg_2, const geometry_msgs::PoseWithCovarianceStamped::ConstPtr est_msg_3){
  ROS_INFO("Estimation");

  flag_callback_estimation = TRUE;
  est_drone1 = *est_msg_1;
  est_drone2 = *est_msg_2;
  est_drone3 = *est_msg_3;
}

void InnovationCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr innov_msg_1, const geometry_msgs::PoseWithCovarianceStamped::ConstPtr innov_msg_2, const geometry_msgs::PoseWithCovarianceStamped::ConstPtr innov_msg_3){
  ROS_INFO("Innovation");

  flag_callback_innovation = TRUE;
  innov_drone = *innov_msg_1;
}

void OdomCallback(const nav_msgs::Odometry::ConstPtr odom_msg_1, const nav_msgs::Odometry::ConstPtr odom_msg_2, const nav_msgs::Odometry::ConstPtr odom_msg_3){
  ROS_INFO("Odom");

  flag_callback_odom = TRUE;
  odom_ptr1 = *odom_msg_1;
  odom_ptr2 = *odom_msg_2;
  odom_ptr3 = *odom_msg_3;
}

//----------------------------------------------------------
//                        Main
//----------------------------------------------------------

int main(int argc, char **argv){
  //----------------------------
  // Init
  ros::init(argc, argv, "evaluation_node");

  //----------------------------
  // Node handler with image transport
  ros::NodeHandle n;

  //----------------------------
  // Suscribers and publishers

  // DKF subscribers
  message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped> sub_estimate1(n, "/quadrotor1/estimate", 1);
  message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped> sub_estimate2(n, "/quadrotor2/estimate", 1);
  message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped> sub_estimate3(n, "/quadrotor3/estimate", 1);

  message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped> sub_inov1(n, "/quadrotor1/innovation", 1);
  message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped> sub_inov2(n, "/quadrotor2/innovation", 1);
  message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped> sub_inov3(n, "/quadrotor3/innovation", 1);

  message_filters::Subscriber<nav_msgs::Odometry> sub_odom1(n, "/quadrotor1/ground_truth/state", 1);
  message_filters::Subscriber<nav_msgs::Odometry> sub_odom2(n, "/quadrotor2/ground_truth/state", 1);
  message_filters::Subscriber<nav_msgs::Odometry> sub_odom3(n, "/quadrotor3/ground_truth/state", 1);

  // Synchroniser policy
  typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseWithCovarianceStamped, geometry_msgs::PoseWithCovarianceStamped, geometry_msgs::PoseWithCovarianceStamped> SyncPolicy;
  message_filters::Synchronizer<SyncPolicy> syncE(SyncPolicy(10), sub_estimate1, sub_estimate2, sub_estimate3);
  syncE.registerCallback(boost::bind(&EstimateCallback, _1, _2, _3));

  //typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseWithCovarianceStamped, geometry_msgs::PoseWithCovarianceStamped, geometry_msgs::PoseWithCovarianceStamped> ISyncPolicy;
  message_filters::Synchronizer<SyncPolicy> syncI(SyncPolicy(10), sub_inov1, sub_inov2, sub_inov3);
  syncI.registerCallback(boost::bind(&InnovationCallback, _1, _2, _3));

  typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, nav_msgs::Odometry, nav_msgs::Odometry> OdomSyncPolicy;
  message_filters::Synchronizer<OdomSyncPolicy> syncO(OdomSyncPolicy(10), sub_odom1, sub_odom2, sub_odom3);
  syncO.registerCallback(boost::bind(&OdomCallback, _1, _2, _3));


  // Neighbours message publishers
  ros::Publisher pub_innov_x = n.advertise<std_msgs::Float64>("innov_x",1000);
  ros::Publisher pub_cov_sup_x = n.advertise<std_msgs::Float64>("cov_sup_x",1000);
  ros::Publisher pub_cov_inf_x = n.advertise<std_msgs::Float64>("cov_inf_x",1000);

  ros::Publisher pub_innov_y = n.advertise<std_msgs::Float64>("innov_y",1000);
  ros::Publisher pub_cov_sup_y = n.advertise<std_msgs::Float64>("cov_sup_y",1000);
  ros::Publisher pub_cov_inf_y = n.advertise<std_msgs::Float64>("cov_inf_y",1000);

  ros::Publisher pub_innov_z = n.advertise<std_msgs::Float64>("innov_z",1000);
  ros::Publisher pub_cov_sup_z = n.advertise<std_msgs::Float64>("cov_sup_z",1000);
  ros::Publisher pub_cov_inf_z = n.advertise<std_msgs::Float64>("cov_inf_z",1000);

  ros::Publisher pub_mean_error = n.advertise<std_msgs::Float64>("mean_error",1000);

  ros::Publisher pub_minimal_distance = n.advertise<std_msgs::Float64>("minimal_distance",1000);
  ros::Publisher pub_target_distance_1 = n.advertise<std_msgs::Float64>("target_distance_1",1000);
  ros::Publisher pub_target_distance_2 = n.advertise<std_msgs::Float64>("target_distance_2",1000);
  ros::Publisher pub_target_distance_3 = n.advertise<std_msgs::Float64>("target_distance_3",1000);

  //----------------------------
  // Init messages

  // Estimation messages
  geometry_msgs::Pose target_pose;
  target_pose.position.x = 5.0; target_pose.position.y = 0.0; target_pose.position.z = 0.5;
  target_pose.orientation.x = 0.0; target_pose.orientation.y = 0.0; target_pose.orientation.z = 0.0; target_pose.orientation.w = 0.0;

  double error_drone1;
  double error_drone2;
  double error_drone3;
  double mean_error;
  std_msgs::Float64 mean_error_msg;

  // Innovation messages
  // x axis innovation
  std_msgs::Float64 x_cov_sup_msg;
  std_msgs::Float64 x_cov_inf_msg;
  std_msgs::Float64 x_inov_msg;

  // y axis innovation
  std_msgs::Float64 y_cov_sup_msg;
  std_msgs::Float64 y_cov_inf_msg;
  std_msgs::Float64 y_inov_msg;

  // z axis innovation
  std_msgs::Float64 z_cov_sup_msg;
  std_msgs::Float64 z_cov_inf_msg;
  std_msgs::Float64 z_inov_msg;

  // Flocking messages
  std_msgs::Float64 minimal_distance;
  std_msgs::Float64 target_distance_1;
  std_msgs::Float64 target_distance_2;
  std_msgs::Float64 target_distance_3;
  
  //----------------------------
  // Rate
  ros::Rate loop_rate(30);

  //----------------------------
  // While loop
  while (ros::ok()){
    if(flag_callback_estimation){
      error_drone1 = getDistance(target_pose, est_drone1.pose.pose);
      error_drone2 = getDistance(target_pose, est_drone2.pose.pose);
      error_drone3 = getDistance(target_pose, est_drone3.pose.pose);
      mean_error = (error_drone1 + error_drone2 + error_drone3)/3.0;
      mean_error_msg.data = mean_error;

      pub_mean_error.publish(mean_error_msg);
    }

    if(flag_callback_innovation){
      // x axis innovation
      x_cov_inf_msg.data = -1.0*innov_drone.pose.covariance[0];
      x_cov_sup_msg.data = innov_drone.pose.covariance[0];
      x_inov_msg.data = innov_drone.pose.pose.position.x;

      // y axis innovation
      y_cov_inf_msg.data = -1.0*innov_drone.pose.covariance[7];
      y_cov_sup_msg.data = innov_drone.pose.covariance[7];
      y_inov_msg.data = innov_drone.pose.pose.position.y;

      // z axis innovation
      z_cov_inf_msg.data = -1.0*innov_drone.pose.covariance[14];
      z_cov_sup_msg.data = innov_drone.pose.covariance[14];
      z_inov_msg.data = innov_drone.pose.pose.position.z;

      // Publish messages
      pub_cov_inf_x.publish(x_cov_inf_msg);
      pub_cov_sup_x.publish(x_cov_sup_msg);
      pub_innov_x.publish(x_inov_msg);

      pub_cov_inf_y.publish(y_cov_inf_msg);
      pub_cov_sup_y.publish(y_cov_sup_msg);
      pub_innov_y.publish(y_inov_msg);

      pub_cov_inf_z.publish(z_cov_inf_msg);
      pub_cov_sup_z.publish(z_cov_sup_msg);
      pub_innov_z.publish(z_inov_msg);
    }

    if(flag_callback_odom){
      minimal_distance.data = std::min(std::min(getDistance(odom_ptr1.pose.pose,odom_ptr2.pose.pose),getDistance(odom_ptr2.pose.pose,odom_ptr3.pose.pose)),getDistance(odom_ptr3.pose.pose,odom_ptr1.pose.pose));
      target_distance_1.data = getDistance(odom_ptr1.pose.pose,target_pose);
      target_distance_2.data = getDistance(odom_ptr2.pose.pose,target_pose);
      target_distance_3.data = getDistance(odom_ptr3.pose.pose,target_pose);

      // Publish messages
      pub_minimal_distance.publish(minimal_distance);
      pub_target_distance_1.publish(target_distance_1);
      pub_target_distance_2.publish(target_distance_2);
      pub_target_distance_3.publish(target_distance_3);
    }

    ros::spinOnce();

    loop_rate.sleep();
  }
  return 0;
}