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
//#include <prototype_6/PoseWithCovarianceArray.h>
#include <aruco_msgs/MarkerArray.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#define FALSE 0
#define TRUE 1
#define TARGET_KEYWORD "Target"
#define SIGMA 0.0
#define MU 0.1
#define RESTRICTED_COM 0
#define COM_RANGE 0.5
#define TARGET_RADIUS 2.0

//----------------------------------------------------------
//                       Global variables
//----------------------------------------------------------


int flag_callback = FALSE;
int flag_callback_drone = FALSE;

geometry_msgs::Pose drone_pose;
aruco_msgs::MarkerArray sensor_poses; 

std::vector<std::vector<geometry_msgs::Pose> > sensors_array;

aruco_msgs::MarkerArray sent_poses_1; 
aruco_msgs::MarkerArray sent_poses_2; 

std_msgs::Header header;

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
void sensorCallback(const aruco_msgs::MarkerArray& msg){

  sensor_poses = msg;
  flag_callback_drone = TRUE;
}

void odomCallback(const geometry_msgs::PoseStamped& msg){

  drone_pose = msg.pose;
  flag_callback_drone = TRUE;
}

void Callback(const aruco_msgs::MarkerArray::ConstPtr sensor_msg, const nav_msgs::Odometry::ConstPtr odom_msg){
  sensor_poses = *sensor_msg;
  drone_pose = odom_msg->pose.pose;
  flag_callback_drone = TRUE;
}


//----------------------------------------------------------
//                        Main
//----------------------------------------------------------

int main(int argc, char **argv)
{
  //----------------------------
  // Init
  ros::init(argc, argv, "sensor_com_node");

  //----------------------------
  // Node handler with image transport
  ros::NodeHandle n;


  //----------------------------
  // Suscribers and publishers
/*
  ros::Subscriber sub_drone = n.subscribe("aruco_marker_publisher/markers", 1, sensorCallback);
  ros::Subscriber sub_odom = n.subscribe("ground_truth/state", 1, odomCallback);
  */
  //----------------------------
  // Suscribers and publishers

  // DKF subscribers
  message_filters::Subscriber<aruco_msgs::MarkerArray> sub_sensor(n, "aruco_marker_publisher/markers", 1);
  message_filters::Subscriber<nav_msgs::Odometry> sub_odom(n, "ground_truth/state", 1);


  // Synchroniser policies
  typedef message_filters::sync_policies::ApproximateTime<aruco_msgs::MarkerArray, nav_msgs::Odometry> SyncPolicy;
  message_filters::Synchronizer<SyncPolicy> syncE(SyncPolicy(10), sub_sensor, sub_odom);
  syncE.registerCallback(boost::bind(&Callback, _1, _2));

  ros::Publisher pub_drone = n.advertise<aruco_msgs::MarkerArray>("corrected_markers",1000);

  //----------------------------
  // Rate
  ros::Rate loop_rate(30);

  //----------------------------
  // Init messages
  tf::TransformListener listener;
  aruco_msgs::MarkerArray targets_msg;

  std::vector<aruco_msgs::Marker> targets_array;

  int counter = 0;
  //----------------------------
  // While loop
  while (ros::ok())
  {

    // Header
    header.frame_id = "nav";
    header.stamp = ros::Time::now();


    if(flag_callback_drone){
      targets_array.clear();
      for(int i=0; i<sensor_poses.markers.size(); i++){
        tf::Transform world_to_drone;
        tf::Transform drone_to_target;
        aruco_msgs::Marker marker;
        marker = sensor_poses.markers[i];


        tf::poseMsgToTF(drone_pose, world_to_drone);
        tf::poseMsgToTF(sensor_poses.markers[i].pose.pose, drone_to_target);
        tf::Transform world_to_target = world_to_drone * drone_to_target;
        tf::poseTFToMsg(world_to_target, marker.pose.pose);

        targets_array.push_back(marker);
      }
      targets_msg.header = header;
      targets_msg.markers = targets_array;
      pub_drone.publish(targets_msg);

    }
 
    
    
    //flag_init = TRUE;
    

    ros::spinOnce();

    loop_rate.sleep();
  }
  return 0;
}
