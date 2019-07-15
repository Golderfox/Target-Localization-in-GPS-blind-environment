#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Header.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Int32.h"


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

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#define FALSE 0
#define TRUE 1
#define TARGET_KEYWORD "Target"
#define SIGMA 0.0
#define MU 0.1
#define NB_TARGETS 3

//----------------------------------------------------------
//                       Global variables
//----------------------------------------------------------


int flag_callback = FALSE;
int flag_callback_drone1 = FALSE;
int flag_callback_drone2 = FALSE;
int flag_callback_drone3 = FALSE;
int flag_callback_drone4 = FALSE;
int flag_callback_drone5 = FALSE;
int flag_callback_drone6 = FALSE;
int flag_callback_mod = FALSE;

std::vector<double> drone_distrib_1;
std::vector<double> drone_distrib_2;
std::vector<double> drone_distrib_3;
std::vector<double> drone_distrib_4;
std::vector<double> drone_distrib_5;
std::vector<double> drone_distrib_6;

std::vector<double> targets_qualities(3, 1.0/3.0);



//----------------------------------------------------------
//                       Callback
//----------------------------------------------------------

/*

void Callback(const std_msgs::Int32::ConstPtr msg_dba1, const std_msgs::Int32::ConstPtr msg_dba2, const std_msgs::Int32::ConstPtr msg_dba3, const std_msgs::Int32::ConstPtr msg_dba4, const std_msgs::Int32::ConstPtr msg_dba5, const std_msgs::Int32::ConstPtr msg_dba6)
{
  ROS_INFO("Odometry");
  drone_affect_1 = msg_dba1->data;
  drone_affect_2 = msg_dba2->data;
  drone_affect_3 = msg_dba3->data;
  drone_affect_4 = msg_dba4->data;
  drone_affect_5 = msg_dba5->data;
  drone_affect_6 = msg_dba6->data;
  flag_callback = TRUE;
}
*/

void droneCallback1(const std_msgs::Float64MultiArray& msg){
  
    drone_distrib_1 = msg.data;
    flag_callback_drone1 = TRUE;
}

void droneCallback2(const std_msgs::Float64MultiArray& msg){
    drone_distrib_2 = msg.data;
    flag_callback_drone2 = TRUE;
}

void droneCallback3(const std_msgs::Float64MultiArray& msg){
    drone_distrib_3 = msg.data;
    flag_callback_drone3 = TRUE;
}

void droneCallback4(const std_msgs::Float64MultiArray& msg){
    drone_distrib_4 = msg.data;
    flag_callback_drone4 = TRUE;
}

void droneCallback5(const std_msgs::Float64MultiArray& msg){
    drone_distrib_5 = msg.data;
    flag_callback_drone5 = TRUE;
}

void droneCallback6(const std_msgs::Float64MultiArray& msg){
    drone_distrib_6 = msg.data;
    flag_callback_drone6 = TRUE;
}
//----------------------------------------------------------
//                        Main
//----------------------------------------------------------

int main(int argc, char **argv)
{
  //----------------------------
  // Init
  ros::init(argc, argv, "eval_node");

  //----------------------------
  // Node handler with image transport
  ros::NodeHandle n;


  //----------------------------
  // Suscribers and publishers
  
  /*
  message_filters::Subscriber<std_msgs::Int32> sub_dba1(n, "/quadrotor1/target_affectation", 1);
  message_filters::Subscriber<std_msgs::Int32> sub_dba2(n, "/quadrotor2/target_affectation", 1);
  message_filters::Subscriber<std_msgs::Int32> sub_dba3(n, "/quadrotor3/target_affectation", 1);
  message_filters::Subscriber<std_msgs::Int32> sub_dba4(n, "/quadrotor4/target_affectation", 1);
  message_filters::Subscriber<std_msgs::Int32> sub_dba5(n, "/quadrotor5/target_affectation", 1);
  message_filters::Subscriber<std_msgs::Int32> sub_dba6(n, "/quadrotor6/target_affectation", 1);
  typedef message_filters::sync_policies::ApproximateTime<std_msgs::Int32, std_msgs::Int32, std_msgs::Int32, std_msgs::Int32, std_msgs::Int32, std_msgs::Int32> SyncPolicy;
  message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(10), sub_dba1, sub_dba2, sub_dba3, sub_dba4, sub_dba5, sub_dba6);
  sync.registerCallback(boost::bind(&Callback, _1, _2, _3, _4, _5, _6));
  */
  
  ros::Subscriber sub_distrib1 = n.subscribe("/quadrotor1/target_distribution", 1, droneCallback1);
  ros::Subscriber sub_distrib2 = n.subscribe("/quadrotor2/target_distribution", 1, droneCallback2);
  ros::Subscriber sub_distrib3 = n.subscribe("/quadrotor3/target_distribution", 1, droneCallback3);
  ros::Subscriber sub_distrib4 = n.subscribe("/quadrotor4/target_distribution", 1, droneCallback4);
  ros::Subscriber sub_distrib5 = n.subscribe("/quadrotor5/target_distribution", 1, droneCallback5);
  ros::Subscriber sub_distrib6 = n.subscribe("/quadrotor6/target_distribution", 1, droneCallback6);

  
  ros::Publisher pub_MAE_1 = n.advertise<std_msgs::Float64>("MAE_drone1",1000);
  ros::Publisher pub_MAE_2 = n.advertise<std_msgs::Float64>("MAE_drone2",1000);
  ros::Publisher pub_MAE_3 = n.advertise<std_msgs::Float64>("MAE_drone3",1000);
  ros::Publisher pub_MAE_4 = n.advertise<std_msgs::Float64>("MAE_drone4",1000);
  ros::Publisher pub_MAE_5 = n.advertise<std_msgs::Float64>("MAE_drone5",1000);
  ros::Publisher pub_MAE_6 = n.advertise<std_msgs::Float64>("MAE_drone6",1000);


  //----------------------------
  // Rate
  ros::Rate loop_rate(30);

  //----------------------------
  // Init msg
  
  // Header
  std_msgs::Header header;
  header.frame_id = "nav";
  header.stamp = ros::Time::now();

  // Init poses
  std_msgs::Float64 MAE_msg_1;
  std_msgs::Float64 MAE_msg_2;
  std_msgs::Float64 MAE_msg_3;
  std_msgs::Float64 MAE_msg_4;
  std_msgs::Float64 MAE_msg_5;
  std_msgs::Float64 MAE_msg_6;

  targets_qualities[0] = 1.0/7.0; targets_qualities[1] = 2.0/7.0; targets_qualities[2] = 4.0/7.0;

  double MAE_1; double MAE_2; double MAE_3; double MAE_4; double MAE_5; double MAE_6; 
  

  //----------------------------
  // While loop
  while (ros::ok())
  { 
      if(flag_callback_drone1){
        ROS_INFO("dr1");
        MAE_1 = 0.0;
        for (int i=0; i<drone_distrib_1.size(); i++){
          //ROS_INFO("dr1c");
            MAE_1 += std::abs(drone_distrib_1[i] - targets_qualities[i]);

        }
        //MAE_1 /= 3.0;
        //ROS_INFO("dr1d");

        // Publish MAE message
        MAE_msg_1.data = MAE_1;
        pub_MAE_1.publish(MAE_msg_1);
        //ROS_INFO("dr1e");
      }

      if(flag_callback_drone2){
        ROS_INFO("dr2");
        MAE_2 = 0.0;
        for (int i=0; i<drone_distrib_2.size(); i++){
            MAE_2 += std::abs(drone_distrib_2[i] - targets_qualities[i]);
        }
        MAE_2 /= targets_qualities.size();

        // Publish MAE message
        MAE_msg_2.data = MAE_2;
        pub_MAE_2.publish(MAE_msg_2);
      }

      if(flag_callback_drone3){
        ROS_INFO("dr3");
        MAE_3 = 0.0;
        for (int i=0; i<drone_distrib_3.size(); i++){
            MAE_3 += std::abs(drone_distrib_3[i] - targets_qualities[i]);
        }
        MAE_3  /= targets_qualities.size();

        // Publish MAE message
        MAE_msg_3.data = MAE_3;
        pub_MAE_3.publish(MAE_msg_3);
      }

      if(flag_callback_drone4){
        ROS_INFO("dr4");
        MAE_4 = 0.0;
        for (int i=0; i<drone_distrib_4.size(); i++){
            MAE_4 += std::abs(drone_distrib_4[i] - targets_qualities[i]);
        }
        MAE_4 /= targets_qualities.size();

        // Publish MAE message
        MAE_msg_4.data = MAE_4;
        pub_MAE_4.publish(MAE_msg_4);
      }

      if(flag_callback_drone5){
        ROS_INFO("dr5");
        MAE_5 = 0.0;
        for (int i=0; i<drone_distrib_5.size(); i++){
            MAE_5 += std::abs(drone_distrib_5[i] - targets_qualities[i]);
        }
        MAE_5 /= targets_qualities.size();

        // Publish MAE message
        MAE_msg_5.data = MAE_5;
        pub_MAE_5.publish(MAE_msg_5);
      }

      if(flag_callback_drone6){
        ROS_INFO("dr6");
        MAE_6 = 0.0;
        for (int i=0; i<drone_distrib_6.size(); i++){
            MAE_6 += std::abs(drone_distrib_6[i] - targets_qualities[i]);
        }
        MAE_6 /= targets_qualities.size();

        // Publish MAE message
        MAE_msg_6.data = MAE_6;
        pub_MAE_6.publish(MAE_msg_6);
      }

    

    ros::spinOnce();

    loop_rate.sleep();
  }
  return 0;
}
