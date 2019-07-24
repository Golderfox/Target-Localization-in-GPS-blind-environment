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
aruco_msgs::MarkerArray incoming_markers_msg;
std_msgs::Header header_msg;


//----------------------------------------------------------
//                       TrackerCallback
//----------------------------------------------------------
void trackerCallback(const aruco_msgs::MarkerArray& msg)
{
  incoming_markers_msg = msg;
  flag_callback_tracker = TRUE;
}

//----------------------------------------------------------
//                        Main
//----------------------------------------------------------

int main(int argc, char **argv)
{
  //----------------------------
  // Init
  ros::init(argc, argv, "sensor_sync");

  //----------------------------
  // Node handler with image transport
  ros::NodeHandle n;


  //----------------------------
  // Suscribers and publishers

  ros::Subscriber sub_sensor = n.subscribe("aruco_marker_publisher/markers", 1, trackerCallback);
  ros::Publisher pub_markers = n.advertise<aruco_msgs::MarkerArray>("sync_markers",1000);
  //----------------------------
  // Rate
  ros::Rate loop_rate(30);

  //----------------------------
  // Init messages
  aruco_msgs::MarkerArray markers_msg;


  //----------------------------
  // While loop
  while (ros::ok())
  {
    // Header
    header_msg.frame_id = "nav";
    header_msg.stamp = ros::Time::now();

    markers_msg.header = header_msg;

    if(flag_callback_tracker){
      
      markers_msg.markers = incoming_markers_msg.markers;

      flag_callback_tracker = FALSE;
    }

    pub_markers.publish(markers_msg);
    ros::spinOnce();

    loop_rate.sleep();
  }
  return 0;
}
