#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Header.h"

#include "stdlib.h"
#include "stdio.h"
#include <string.h>
#include <cmath>
#include <random_numbers/random_numbers.h>
#include "tf/transform_datatypes.h"
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseArray.h>

#include <aruco_msgs/MarkerArray.h>

#include <prototype_6/PoseWithCovarianceArray.h>

#define FALSE 0
#define TRUE 1
#define TARGET_KEYWORD "Target"
#define SIGMA 0.0
#define MU 0.1
#define RESTRICTED_COM 0
#define COM_RANGE 0.5
#define TARGET_RADIUS 1.0

//----------------------------------------------------------
//                       Global variables
//----------------------------------------------------------


int flag_callback_tracker = FALSE;

std::vector<geometry_msgs::PoseWithCovariance> sensor_array;
std_msgs::Header header_msg;


//----------------------------------------------------------
//                       TrackerCallback
//----------------------------------------------------------
void trackerCallback(const aruco_msgs::MarkerArray& msg)
{
  header_msg = msg.header;
  sensor_array.clear();
  for (int i=0; msg.markers.size(); i++){
    sensor_array.push_back(msg.markers[i].pose);
  }
  flag_callback_tracker = TRUE;
}

//----------------------------------------------------------
//                        Main
//----------------------------------------------------------

int main(int argc, char **argv)
{
  //----------------------------
  // Init
  ros::init(argc, argv, "sensor_conversion");

  //----------------------------
  // Node handler with image transport
  ros::NodeHandle n;


  //----------------------------
  // Suscribers and publishers

  ros::Subscriber sub_sensor = n.subscribe("aruco_marker/state", 1, trackerCallback);
  ros::Publisher pub_poses = n.advertise<prototype_6::PoseWithCovarianceArray>("converted_tracker",1000);
  //----------------------------
  // Rate
  ros::Rate loop_rate(30);

  //----------------------------
  // Init messages
  prototype_6::PoseWithCovarianceArray markers_msg;


  //----------------------------
  // While loop
  while (ros::ok())
  {
    if(flag_callback_tracker){
      markers_msg.header = header_msg;
      markers_msg.poses = sensor_array;

      pub_poses.publish(markers_msg);
      flag_callback_tracker = FALSE;
    }
    ros::spinOnce();

    loop_rate.sleep();
  }
  return 0;
}
