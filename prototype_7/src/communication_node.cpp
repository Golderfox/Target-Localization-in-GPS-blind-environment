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

#include <prototype_7/DKF.h>
#include <prototype_7/DKFStamped.h>
#include <prototype_7/DKF_multi.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#define FALSE 0
#define TRUE 1
#define COM_RANGE 0.5
#define TARGET_RADIUS 1.5


//----------------------------------------------------------
//                    Defining global variables
//----------------------------------------------------------

int flag_callback = FALSE;

prototype_7::DKF drone_out_1;
prototype_7::DKF drone_out_2;


std_msgs::Header img_header;


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

geometry_msgs::Pose DKFToPose(prototype_7::DKF msg){
  std::cout << "func" <<std::endl;
  std::cout << msg.y[0] <<std::endl;
  std::cout << msg.y[1] <<std::endl;
  std::cout << msg.y[2] <<std::endl;
  geometry_msgs::Pose output_pose;
  output_pose.position.x = msg.y[0]; 
  output_pose.position.y = msg.y[1];
  output_pose.position.z = msg.y[2];
  output_pose.orientation.x = 0.0;
  output_pose.orientation.y = 0.0;
  output_pose.orientation.z = 0.0;
  output_pose.orientation.w = 0.0;
  return output_pose;
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

void droneCallback(const prototype_7::DKFStamped::ConstPtr drone_msg_1, const prototype_7::DKFStamped::ConstPtr drone_msg_2){
  ROS_INFO("Odometry");
  drone_out_1 = drone_msg_1->data;
  drone_out_2 = drone_msg_2->data;
  flag_callback = TRUE;
}


//----------------------------------------------------------
//                        Main
//----------------------------------------------------------

int main(int argc, char **argv){
  //----------------------------
  // Init
  std::cout << "init" <<std::endl;
  ros::init(argc, argv, "communication_node");
  //----------------------------
  // Node handler with image transport
  ros::NodeHandle n;

  //----------------------------
  // Suscribers and publishers

  // DKF subscribers
  message_filters::Subscriber<prototype_7::DKFStamped> sub_com1(n, "/quadrotor1/output_com", 1);
  message_filters::Subscriber<prototype_7::DKFStamped> sub_com2(n, "/quadrotor0/output_com", 1);


  // Synchroniser policy
  typedef message_filters::sync_policies::ApproximateTime<prototype_7::DKFStamped, prototype_7::DKFStamped> SyncPolicy;
  message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(10), sub_com1, sub_com2);
  sync.registerCallback(boost::bind(&droneCallback, _1, _2));

  /*
  // Neighbours message subscribers
  ros::Subscriber sub_input1 = n.subscribe("/quadrotor1/DKF_out", 1, drone1Callback);
  ros::Subscriber sub_input2 = n.subscribe("/quadrotor2/DKF_out", 1, drone2Callback);
  ros::Subscriber sub_input3 = n.subscribe("/quadrotor3/DKF_out", 1, drone3Callback);
  */

  // Neighbours message publishers
  ros::Publisher pub_output1 = n.advertise<prototype_7::DKF_multi>("/quadrotor1/input_com",1000);
  ros::Publisher pub_output2 = n.advertise<prototype_7::DKF_multi>("/quadrotor0/input_com",1000);

  //----------------------------
  // Init messages
  prototype_7::DKF_multi drone_input_1;
  prototype_7::DKF_multi drone_input_2;

  std::vector<prototype_7::DKF> DKF_array_1;
  std::vector<prototype_7::DKF> DKF_array_2;


  geometry_msgs::Pose sensor_pose_1;
  geometry_msgs::Pose sensor_pose_2;

  

  //----------------------------
  // Rate
  ros::Rate loop_rate(30);

  //----------------------------
  // While loop
  while (ros::ok()){

    // Clear DKF arrays
    DKF_array_1.clear();
    DKF_array_2.clear();


    //----------------------------
    // If callback is received
    if (flag_callback == TRUE){
      std::cout << "callback" <<std::endl;
      // Update sensors poses
      if(drone_out_1.y_size !=0){
        sensor_pose_1 = DKFToPose(drone_out_1);
      }
      if(drone_out_2.y_size !=0){
        sensor_pose_2 = DKFToPose(drone_out_2);
      }


      //----------------------------
      // Assign message to drones

      // Check link 1-2
      
      if(getDistance(drone_out_1.allocation, drone_out_2.allocation) < TARGET_RADIUS){
        if(drone_out_2.y_size !=0){
          DKF_array_1.push_back(drone_out_2);
        }
        if(drone_out_1.y_size !=0){
          DKF_array_2.push_back(drone_out_1);
        }
      }
      
      drone_input_1.array = DKF_array_1;
      drone_input_2.array = DKF_array_2;


      // Publish messages to drones od the swarm
      pub_output1.publish(drone_input_1);
      pub_output2.publish(drone_input_2);


    }
    std::cout << "loop" <<std::endl;
  
    ros::spinOnce();

    loop_rate.sleep();
  }
  return 0;
}