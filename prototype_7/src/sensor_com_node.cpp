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
int flag_callback_drone_1 = FALSE;
int flag_callback_drone_2 = FALSE;
int flag_init = FALSE;


aruco_msgs::MarkerArray sensor_poses_1; 
aruco_msgs::MarkerArray sensor_poses_2; 

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
void droneCallback1(const aruco_msgs::MarkerArray& msg){

  sensor_poses_1 = msg;
  flag_callback_drone_1 = TRUE;
}

void droneCallback2(const aruco_msgs::MarkerArray& msg){
  sensor_poses_2 = msg;
  flag_callback_drone_2 = TRUE;
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

  ros::Subscriber sub_drone_1 = n.subscribe("/quadrotor1/aruco_marker_publisher/markers", 1, droneCallback1);
  ros::Subscriber sub_drone_2 = n.subscribe("/quadrotor0/aruco_marker_publisher/markers", 1, droneCallback2);


  ros::Publisher pub_drone_1 = n.advertise<aruco_msgs::MarkerArray>("/quadrotor1/neighbours_sensors",1000);
  ros::Publisher pub_drone_2 = n.advertise<aruco_msgs::MarkerArray>("/quadrotor0/neighbours_sensors",1000);

  //----------------------------
  // Rate
  ros::Rate loop_rate(30);

  //----------------------------
  // Init messages
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


    if(flag_callback_drone_1){
      flag_init = TRUE;
      std::cout << "callback 1" << std::endl;
      for(int i=0; i<sensor_poses_1.markers.size(); i++){
        int flag_new = TRUE;
        for(int j=0; j<targets_array.size(); j++){
          if(getDistance(sensor_poses_1.markers[i].pose.pose,targets_array[j].pose.pose) < TARGET_RADIUS){
            targets_array[j] = sensor_poses_1.markers[i];
            flag_new = FALSE;
            break;
          }
        }
        if(flag_new){
          targets_array.push_back(sensor_poses_1.markers[i]);
        }
      }
    }
    

    if(flag_callback_drone_2){
      flag_init = TRUE;
      std::cout << "callback 2" << std::endl;
      for(int i=0; i<sensor_poses_2.markers.size(); i++){
        int flag_new = TRUE;
        for(int j=0; j<targets_array.size(); j++){
          if(getDistance(sensor_poses_2.markers[i].pose.pose,targets_array[j].pose.pose) < TARGET_RADIUS){
            targets_array[j] = sensor_poses_2.markers[i];
            flag_new = FALSE;
            break;
          }
        }
        if(flag_new){
          targets_array.push_back(sensor_poses_2.markers[i]);
        }
      }
    }
    

   
    
    if(counter > 5){

      targets_msg.header = header;
      targets_msg.markers = targets_array;
      pub_drone_1.publish(targets_msg);
      pub_drone_2.publish(targets_msg);
    }
    if(flag_init){
      counter += 1;
    }
    
    
    //flag_init = TRUE;
    

    ros::spinOnce();

    loop_rate.sleep();
  }
  return 0;
}
