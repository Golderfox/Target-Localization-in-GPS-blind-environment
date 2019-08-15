#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float64.h"
#include <std_msgs/Float32MultiArray.h>
#include "std_msgs/Header.h"

#include <nav_msgs/Odometry.h>


#include "stdlib.h"
#include "stdio.h"
#include <string.h>
#include <cmath>
#include <complex>
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

#include <aruco_msgs/MarkerArray.h>

#define FALSE 0
#define TRUE 1
#define COM_RANGE 1.5
#define TARGET_RADIUS 1.5

//----------------------------------------------------------
//                       Global variables
//----------------------------------------------------------


int flag_callback_estimation = FALSE;
int flag_callback_sensor = FALSE;
int flag_callback_innovation = FALSE;
int flag_callback_odom = FALSE;
int flag_callback_mod = FALSE;
int flag_callback_tag = FALSE;

geometry_msgs::PoseWithCovarianceStamped est_drone1;
geometry_msgs::PoseWithCovarianceStamped est_drone2;

geometry_msgs::PoseWithCovarianceStamped sensor_drone1;
geometry_msgs::PoseWithCovarianceStamped sensor_drone2;

geometry_msgs::Pose est_target_pose;
geometry_msgs::Pose tag_pose;

std::vector<geometry_msgs::Pose> est_array;

std::vector<geometry_msgs::Pose> sensor_array;

geometry_msgs::PoseWithCovarianceStamped innov_drone;

nav_msgs::Odometry odom_ptr1;
nav_msgs::Odometry odom_ptr2;

geometry_msgs::Pose odom_target_pose;

std::vector<nav_msgs::Odometry> odom_array;


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


void EstimateCallback(const geometry_msgs::PoseWithCovarianceStamped& msg){
  ROS_INFO("Estimation");
  est_array.clear();
  flag_callback_estimation = TRUE;
  est_drone1 = msg;
}

void SensorCallback(const aruco_msgs::MarkerArray& msg){
  ROS_INFO("Sensor");
  flag_callback_sensor = TRUE;
  sensor_drone1.header = msg.header;
  sensor_drone1.pose = msg.markers[0].pose;

}


void tagCallback(const geometry_msgs::TransformStamped& msg){
  ROS_INFO("Tag");
  flag_callback_tag = TRUE;
  tag_pose.position.x = msg.transform.translation.x;
  tag_pose.position.y = msg.transform.translation.y;
  tag_pose.position.z = msg.transform.translation.z;

  tag_pose.orientation.x = msg.transform.rotation.x;
  tag_pose.orientation.y = msg.transform.rotation.y;
  tag_pose.orientation.z = msg.transform.rotation.z;
  tag_pose.orientation.w = msg.transform.rotation.w;

}

//----------------------------------------------------------
//                        Main
//----------------------------------------------------------

int main(int argc, char **argv){
  //----------------------------
  // Init
  std::cout << "init" << std::endl;
  ros::init(argc, argv, "evaluation_node");

  //----------------------------
  // Node handler with image transport
  ros::NodeHandle n;

  //----------------------------
  // Suscribers and publishers

  // DKF subscribers

  /*
  // Odometry subscribers
  message_filters::Subscriber<nav_msgs::Odometry> sub_odom1(n, "/quadrotor1/ground_truth/state", 1);
  message_filters::Subscriber<nav_msgs::Odometry> sub_odom2(n, "/quadrotor0/ground_truth/state", 1);
  message_filters::Subscriber<geometry_msgs::PoseStamped> sub_target_odom(n, "/target", 1);
  */
  ros::Subscriber sub_sensor = n.subscribe("/quadrotor0/aruco_marker_publisher/markers", 1, SensorCallback);
  ros::Subscriber sub_estimate = n.subscribe("/quadrotor0/estimate", 1, EstimateCallback);
  ros::Subscriber sub_tag = n.subscribe("/vicon/tag/tag", 1, tagCallback);

  

  /*  
  typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, nav_msgs::Odometry, geometry_msgs::PoseStamped> OdomSyncPolicy;
  message_filters::Synchronizer<OdomSyncPolicy> syncO(OdomSyncPolicy(10), sub_odom1, sub_odom2, sub_target_odom);
  syncO.registerCallback(boost::bind(&OdomCallback, _1, _2, _3));
  */
  // Neighbours message publishers
  ros::Publisher pub_mean_error = n.advertise<std_msgs::Float64>("mean_error",1000);
  ros::Publisher pub_mean_sensor_error = n.advertise<std_msgs::Float64>("mean_sensor_error",1000);


  /*
  ros::Publisher pub_minimal_distance = n.advertise<std_msgs::Float64>("minimal_distance",1000);
  ros::Publisher pub_target_distance_1 = n.advertise<std_msgs::Float64>("target_distance_1",1000);
  ros::Publisher pub_target_distance_2 = n.advertise<std_msgs::Float64>("target_distance_2",1000);
  */

  /*
  ros::Publisher pub_mean_velocity = n.advertise<std_msgs::Float32MultiArray>("mean_velocity",1000);
  ros::Publisher pub_biggest_cluster = n.advertise<std_msgs::Int32>("biggest_cluster",1000);
  ros::Publisher pub_order = n.advertise<std_msgs::Float32MultiArray>("order",1000);
  */

  

  //----------------------------
  // Init targets
  std::vector< std::vector<int> > subswarms_indexes;
  std::vector<int> targets_indexes;
  geometry_msgs::Pose target_pose_1; geometry_msgs::Pose target_pose_2; geometry_msgs::Pose target_pose_3;
  std::vector<geometry_msgs::Pose> targets_poses;
  // Target 1
  target_pose_1.position.x = 5.0; target_pose_1.position.y = 0.0; target_pose_1.position.z = 0.5;
  target_pose_1.orientation.x = 0.0; target_pose_1.orientation.y = 0.0; target_pose_1.orientation.z = 0.0; target_pose_1.orientation.w = 0.0;
  targets_poses.push_back(target_pose_1);


  //----------------------------
  // Init messages

  // Init variables
  double error_drone1;
  double error_drone2;

  double error_sensor1;
  double error_sensor2;

  double mean_error;
  double mean_sensor_error;
  double mean_velocity;
  int bigger_cluster;

  std::vector<double> distances_swarm;
  std::vector<float> mean_velocities;

  std::vector<float> orders;

  // Error messages
  std_msgs::Float64 mean_error_msg;
  std_msgs::Float64 error_1_msg;
  std_msgs::Float64 error_2_msg;
  std_msgs::Float64 sensor_mean_error_msg;

  // Flocking messages
  std_msgs::Float64 minimal_distance;
  std_msgs::Float64 target_distance_1;
  std_msgs::Float64 target_distance_2;

  std_msgs::Float32MultiArray order_msg;
  
  //----------------------------
  // Rate
  ros::Rate loop_rate(30);

  //----------------------------
  // While loop
  while (ros::ok()){
    
    if(flag_callback_estimation && flag_callback_tag){

      // Generate error messages

      mean_error = getDistance(tag_pose, est_drone1.pose.pose);
      mean_error_msg.data = mean_error;

      pub_mean_error.publish(mean_error_msg);
    }

    if(flag_callback_sensor && flag_callback_tag){
      error_sensor1 = getDistance(tag_pose, sensor_drone1.pose.pose);
      error_sensor2 = getDistance(tag_pose, sensor_drone2.pose.pose);
      

      mean_sensor_error = (error_sensor1 + error_sensor2)/2.0;
      ROS_INFO("value: %f", mean_sensor_error);
      sensor_mean_error_msg.data = mean_sensor_error;
      pub_mean_sensor_error.publish(sensor_mean_error_msg);
    }
  

    ros::spinOnce();

    loop_rate.sleep();
  }
  return 0;
}