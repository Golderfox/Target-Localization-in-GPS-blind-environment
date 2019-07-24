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
geometry_msgs::Pose drone_pose_3;
geometry_msgs::Pose drone_pose_4;
geometry_msgs::Pose drone_pose_5;
geometry_msgs::Pose drone_pose_6;

std_msgs::Header drone_header;


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

void droneCallback(const geometry_msgs::PoseStamped::ConstPtr drone_msg_1, const geometry_msgs::PoseStamped::ConstPtr drone_msg_2, const geometry_msgs::PoseStamped::ConstPtr drone_msg_3, const geometry_msgs::PoseStamped::ConstPtr drone_msg_4, const geometry_msgs::PoseStamped::ConstPtr drone_msg_5, const geometry_msgs::PoseStamped::ConstPtr drone_msg_6){
  ROS_INFO("Odometry");
  drone_header = drone_msg_1->header;
  drone_pose_1 = drone_msg_1->pose;
  drone_pose_2 = drone_msg_2->pose;
  drone_pose_3 = drone_msg_3->pose;
  drone_pose_4 = drone_msg_4->pose;
  drone_pose_5 = drone_msg_5->pose;
  drone_pose_6 = drone_msg_6->pose;
  flag_callback_drone = TRUE;
}


//----------------------------------------------------------
//                        Main
//----------------------------------------------------------

int main(int argc, char **argv){
  //----------------------------
  // Init
  ros::init(argc, argv, "swarm_node");

  //----------------------------
  // Node handler with image transport
  ros::NodeHandle n;

  //----------------------------
  // Suscribers and publishers

  // DKF subscribers
  message_filters::Subscriber<geometry_msgs::PoseStamped> sub_odom1(n, "/quadrotor1/drone_pose", 1);
  message_filters::Subscriber<geometry_msgs::PoseStamped> sub_odom2(n, "/quadrotor2/drone_pose", 1);
  message_filters::Subscriber<geometry_msgs::PoseStamped> sub_odom3(n, "/quadrotor3/drone_pose", 1);
  message_filters::Subscriber<geometry_msgs::PoseStamped> sub_odom4(n, "/quadrotor4/drone_pose", 1);
  message_filters::Subscriber<geometry_msgs::PoseStamped> sub_odom5(n, "/quadrotor5/drone_pose", 1);
  message_filters::Subscriber<geometry_msgs::PoseStamped> sub_odom6(n, "/quadrotor6/drone_pose", 1);

  // Synchroniser policy
  typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseStamped, geometry_msgs::PoseStamped, geometry_msgs::PoseStamped, geometry_msgs::PoseStamped, geometry_msgs::PoseStamped, geometry_msgs::PoseStamped> SyncPolicy;
  message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(10), sub_odom1, sub_odom2, sub_odom3, sub_odom4, sub_odom5, sub_odom6);
  sync.registerCallback(boost::bind(&droneCallback, _1, _2, _3, _4, _5, _6));

  /*
  // Neighbours message subscribers
  ros::Subscriber sub_input1 = n.subscribe("/quadrotor1/DKF_out", 1, drone1Callback);
  ros::Subscriber sub_input2 = n.subscribe("/quadrotor2/DKF_out", 1, drone2Callback);
  ros::Subscriber sub_input3 = n.subscribe("/quadrotor3/DKF_out", 1, drone3Callback);
  */

  // Neighbours message publishers
  ros::Publisher pub_output1 = n.advertise<geometry_msgs::PoseArray>("/quadrotor1/swarm",1000);
  ros::Publisher pub_output2 = n.advertise<geometry_msgs::PoseArray>("/quadrotor2/swarm",1000);
  ros::Publisher pub_output3 = n.advertise<geometry_msgs::PoseArray>("/quadrotor3/swarm",1000);
  ros::Publisher pub_output4 = n.advertise<geometry_msgs::PoseArray>("/quadrotor4/swarm",1000);
  ros::Publisher pub_output5 = n.advertise<geometry_msgs::PoseArray>("/quadrotor5/swarm",1000);
  ros::Publisher pub_output6 = n.advertise<geometry_msgs::PoseArray>("/quadrotor6/swarm",1000);

  //----------------------------
  // Init messages
  std::vector<geometry_msgs::Pose> drone_neighbours_1;
  std::vector<geometry_msgs::Pose> drone_neighbours_2;
  std::vector<geometry_msgs::Pose> drone_neighbours_3;
  std::vector<geometry_msgs::Pose> drone_neighbours_4;
  std::vector<geometry_msgs::Pose> drone_neighbours_5;
  std::vector<geometry_msgs::Pose> drone_neighbours_6;

  geometry_msgs::PoseArray drone_neighbours_msg_1;
  geometry_msgs::PoseArray drone_neighbours_msg_2;
  geometry_msgs::PoseArray drone_neighbours_msg_3;
  geometry_msgs::PoseArray drone_neighbours_msg_4;
  geometry_msgs::PoseArray drone_neighbours_msg_5;
  geometry_msgs::PoseArray drone_neighbours_msg_6;

  //----------------------------
  // Rate
  ros::Rate loop_rate(30);

  //----------------------------
  // While loop
  while (ros::ok()){

    // Clear DKF arrays
    drone_neighbours_1.clear();
    drone_neighbours_2.clear();
    drone_neighbours_3.clear();
    drone_neighbours_4.clear();
    drone_neighbours_5.clear();
    drone_neighbours_6.clear();

    //----------------------------
    // If callback is received
    if (flag_callback_drone == TRUE){

      // Assign message to drone 1
      drone_neighbours_1.push_back(drone_pose_2);
      drone_neighbours_1.push_back(drone_pose_3);
      drone_neighbours_1.push_back(drone_pose_4);
      drone_neighbours_1.push_back(drone_pose_5);
      drone_neighbours_1.push_back(drone_pose_6);

      // Assign message to drone 2
      drone_neighbours_2.push_back(drone_pose_1);
      drone_neighbours_2.push_back(drone_pose_3);
      drone_neighbours_2.push_back(drone_pose_4);
      drone_neighbours_2.push_back(drone_pose_5);
      drone_neighbours_2.push_back(drone_pose_6);

      // Assign message to drone 3
      drone_neighbours_3.push_back(drone_pose_1);
      drone_neighbours_3.push_back(drone_pose_2);
      drone_neighbours_3.push_back(drone_pose_4);
      drone_neighbours_3.push_back(drone_pose_5);
      drone_neighbours_3.push_back(drone_pose_6);

      // Assign message to drone 4
      drone_neighbours_4.push_back(drone_pose_1);
      drone_neighbours_4.push_back(drone_pose_2);
      drone_neighbours_4.push_back(drone_pose_3);
      drone_neighbours_4.push_back(drone_pose_5);
      drone_neighbours_4.push_back(drone_pose_6);

      // Assign message to drone 5
      drone_neighbours_5.push_back(drone_pose_1);
      drone_neighbours_5.push_back(drone_pose_2);
      drone_neighbours_5.push_back(drone_pose_3);
      drone_neighbours_5.push_back(drone_pose_4);
      drone_neighbours_5.push_back(drone_pose_6);

      // Assign message to drone 6
      drone_neighbours_6.push_back(drone_pose_1);
      drone_neighbours_6.push_back(drone_pose_2);
      drone_neighbours_6.push_back(drone_pose_3);
      drone_neighbours_6.push_back(drone_pose_4);
      drone_neighbours_6.push_back(drone_pose_5);

      // Complete messages 
      drone_neighbours_msg_1.header = drone_header;
      drone_neighbours_msg_1.poses = drone_neighbours_1;
      drone_neighbours_msg_2.header = drone_header;
      drone_neighbours_msg_2.poses = drone_neighbours_2;
      drone_neighbours_msg_3.header = drone_header;
      drone_neighbours_msg_3.poses = drone_neighbours_3;
      drone_neighbours_msg_4.header = drone_header;
      drone_neighbours_msg_4.poses = drone_neighbours_4;
      drone_neighbours_msg_5.header = drone_header;
      drone_neighbours_msg_5.poses = drone_neighbours_5;
      drone_neighbours_msg_6.header = drone_header;
      drone_neighbours_msg_6.poses = drone_neighbours_6;

      // Publish messages to drones od the swarm
      pub_output1.publish(drone_neighbours_msg_1);
      pub_output2.publish(drone_neighbours_msg_2);
      pub_output3.publish(drone_neighbours_msg_3);
      pub_output4.publish(drone_neighbours_msg_4);
      pub_output5.publish(drone_neighbours_msg_5);
      pub_output6.publish(drone_neighbours_msg_6);
    }
  
    
    ros::spinOnce();

    loop_rate.sleep();
  }
  return 0;
}