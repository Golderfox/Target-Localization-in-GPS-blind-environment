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

#include <prototype_6/DKF.h>
#include <prototype_6/DKFStamped.h>
#include <prototype_6/DKF_multi.h>

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

prototype_6::DKF drone_out_1;
prototype_6::DKF drone_out_2;
prototype_6::DKF drone_out_3;
prototype_6::DKF drone_out_4;
prototype_6::DKF drone_out_5;
prototype_6::DKF drone_out_6;

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

geometry_msgs::Pose DKFToPose(prototype_6::DKF msg){
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

void droneCallback(const prototype_6::DKFStamped::ConstPtr drone_msg_1, const prototype_6::DKFStamped::ConstPtr drone_msg_2, const prototype_6::DKFStamped::ConstPtr drone_msg_3, const prototype_6::DKFStamped::ConstPtr drone_msg_4, const prototype_6::DKFStamped::ConstPtr drone_msg_5, const prototype_6::DKFStamped::ConstPtr drone_msg_6){
  ROS_INFO("Odometry");
  drone_out_1 = drone_msg_1->data;
  drone_out_2 = drone_msg_2->data;
  drone_out_3 = drone_msg_3->data;
  drone_out_4 = drone_msg_4->data;
  drone_out_5 = drone_msg_5->data;
  drone_out_6 = drone_msg_6->data;
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
  message_filters::Subscriber<prototype_6::DKFStamped> sub_com1(n, "/quadrotor1/output_com", 1);
  message_filters::Subscriber<prototype_6::DKFStamped> sub_com2(n, "/quadrotor2/output_com", 1);
  message_filters::Subscriber<prototype_6::DKFStamped> sub_com3(n, "/quadrotor3/output_com", 1);
  message_filters::Subscriber<prototype_6::DKFStamped> sub_com4(n, "/quadrotor4/output_com", 1);
  message_filters::Subscriber<prototype_6::DKFStamped> sub_com5(n, "/quadrotor5/output_com", 1);
  message_filters::Subscriber<prototype_6::DKFStamped> sub_com6(n, "/quadrotor6/output_com", 1);

  // Synchroniser policy
  typedef message_filters::sync_policies::ApproximateTime<prototype_6::DKFStamped, prototype_6::DKFStamped, prototype_6::DKFStamped, prototype_6::DKFStamped, prototype_6::DKFStamped, prototype_6::DKFStamped> SyncPolicy;
  message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(10), sub_com1, sub_com2, sub_com3, sub_com4, sub_com5, sub_com6);
  sync.registerCallback(boost::bind(&droneCallback, _1, _2, _3, _4, _5, _6));

  /*
  // Neighbours message subscribers
  ros::Subscriber sub_input1 = n.subscribe("/quadrotor1/DKF_out", 1, drone1Callback);
  ros::Subscriber sub_input2 = n.subscribe("/quadrotor2/DKF_out", 1, drone2Callback);
  ros::Subscriber sub_input3 = n.subscribe("/quadrotor3/DKF_out", 1, drone3Callback);
  */

  // Neighbours message publishers
  ros::Publisher pub_output1 = n.advertise<prototype_6::DKF_multi>("/quadrotor1/input_com",1000);
  ros::Publisher pub_output2 = n.advertise<prototype_6::DKF_multi>("/quadrotor2/input_com",1000);
  ros::Publisher pub_output3 = n.advertise<prototype_6::DKF_multi>("/quadrotor3/input_com",1000);
  ros::Publisher pub_output4 = n.advertise<prototype_6::DKF_multi>("/quadrotor4/input_com",1000);
  ros::Publisher pub_output5 = n.advertise<prototype_6::DKF_multi>("/quadrotor5/input_com",1000);
  ros::Publisher pub_output6 = n.advertise<prototype_6::DKF_multi>("/quadrotor6/input_com",1000);

  //----------------------------
  // Init messages
  prototype_6::DKF_multi drone_input_1;
  prototype_6::DKF_multi drone_input_2;
  prototype_6::DKF_multi drone_input_3;
  prototype_6::DKF_multi drone_input_4;
  prototype_6::DKF_multi drone_input_5;
  prototype_6::DKF_multi drone_input_6;

  std::vector<prototype_6::DKF> DKF_array_1;
  std::vector<prototype_6::DKF> DKF_array_2;
  std::vector<prototype_6::DKF> DKF_array_3;
  std::vector<prototype_6::DKF> DKF_array_4;
  std::vector<prototype_6::DKF> DKF_array_5;
  std::vector<prototype_6::DKF> DKF_array_6;

  geometry_msgs::Pose sensor_pose_1;
  geometry_msgs::Pose sensor_pose_2;
  geometry_msgs::Pose sensor_pose_3;
  geometry_msgs::Pose sensor_pose_4;
  geometry_msgs::Pose sensor_pose_5;
  geometry_msgs::Pose sensor_pose_6;
  

  //----------------------------
  // Rate
  ros::Rate loop_rate(30);

  //----------------------------
  // While loop
  while (ros::ok()){

    // Clear DKF arrays
    DKF_array_1.clear();
    DKF_array_2.clear();
    DKF_array_3.clear();
    DKF_array_4.clear();
    DKF_array_5.clear();
    DKF_array_6.clear();

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
      if(drone_out_3.y_size !=0){
        sensor_pose_3 = DKFToPose(drone_out_3);
      }
      if(drone_out_4.y_size !=0){
        sensor_pose_4 = DKFToPose(drone_out_4);
      }
      if(drone_out_5.y_size !=0){
        sensor_pose_5 = DKFToPose(drone_out_5);
      }
      if(drone_out_6.y_size !=0){
        sensor_pose_6 = DKFToPose(drone_out_6);
      }

      //----------------------------
      // Assign message to drones
      std::cout << "assign arrays" <<std::endl;
      // Check link 1-2
      
      if(getDistance(drone_out_1.allocation, drone_out_2.allocation) < TARGET_RADIUS){
        if(drone_out_2.y_size !=0){
          DKF_array_1.push_back(drone_out_2);
        }
        if(drone_out_1.y_size !=0){
          DKF_array_2.push_back(drone_out_1);
        }
      }
      

      // Check link 1-3
      
      if(getDistance(drone_out_1.allocation, drone_out_3.allocation) < TARGET_RADIUS){
        if(drone_out_3.y_size !=0){
          DKF_array_1.push_back(drone_out_3);
        }
        if(drone_out_1.y_size !=0){
          DKF_array_3.push_back(drone_out_1);
        }
      }

      // Check link 1-4
      if(getDistance(drone_out_1.allocation, drone_out_4.allocation) < TARGET_RADIUS){
        if(drone_out_4.y_size !=0){
          DKF_array_1.push_back(drone_out_4);
        }
        if(drone_out_1.y_size !=0){
          DKF_array_4.push_back(drone_out_1);
        }
      }

      // Check link 1-5
      if(getDistance(drone_out_1.allocation, drone_out_5.allocation) < TARGET_RADIUS){
        if(drone_out_5.y_size !=0){
          DKF_array_1.push_back(drone_out_5);
        }
        if(drone_out_1.y_size !=0){
          DKF_array_5.push_back(drone_out_1);
        }
      }

      // Check link 1-6
      if(getDistance(drone_out_1.allocation, drone_out_6.allocation) < TARGET_RADIUS){
        if(drone_out_6.y_size !=0){
          DKF_array_1.push_back(drone_out_6);
        }
        if(drone_out_1.y_size !=0){
          DKF_array_6.push_back(drone_out_1);
        }
      }

      // Check link 2-3
      if(getDistance(drone_out_2.allocation, drone_out_3.allocation) < TARGET_RADIUS){
        if(drone_out_3.y_size !=0){
          DKF_array_2.push_back(drone_out_3);
        }
        if(drone_out_2.y_size !=0){
          DKF_array_3.push_back(drone_out_2);
        }
      }

      // Check link 2-4
      if(getDistance(drone_out_2.allocation, drone_out_4.allocation) < TARGET_RADIUS){
        if(drone_out_4.y_size !=0){
          DKF_array_2.push_back(drone_out_4);
        }
        if(drone_out_2.y_size !=0){
          DKF_array_4.push_back(drone_out_2);
        }
      }

      // Check link 2-5
      if(getDistance(drone_out_2.allocation, drone_out_5.allocation) < TARGET_RADIUS){
        if(drone_out_5.y_size !=0){
          DKF_array_2.push_back(drone_out_5);
        }
        if(drone_out_2.y_size !=0){
          DKF_array_5.push_back(drone_out_2);
        }
      }

      // Check link 2-6
      if(getDistance(drone_out_2.allocation, drone_out_6.allocation) < TARGET_RADIUS){
        if(drone_out_6.y_size !=0){
          DKF_array_2.push_back(drone_out_6);
        }
        if(drone_out_2.y_size !=0){
          DKF_array_6.push_back(drone_out_2);
        }
      }

      // Check link 3-4
      if(getDistance(drone_out_3.allocation, drone_out_4.allocation) < TARGET_RADIUS){
        if(drone_out_4.y_size !=0){
          DKF_array_3.push_back(drone_out_4);
        }
        if(drone_out_3.y_size !=0){
          DKF_array_4.push_back(drone_out_3);
        }
      }

      // Check link 3-5
      if(getDistance(drone_out_3.allocation, drone_out_5.allocation) < TARGET_RADIUS){
        if(drone_out_5.y_size !=0){
          DKF_array_3.push_back(drone_out_5);
        }
        if(drone_out_3.y_size !=0){
          DKF_array_5.push_back(drone_out_3);
        }
      }

      // Check link 3-6
      if(getDistance(drone_out_3.allocation, drone_out_6.allocation) < TARGET_RADIUS){
        if(drone_out_6.y_size !=0){
          DKF_array_3.push_back(drone_out_6);
        }
        if(drone_out_3.y_size !=0){
          DKF_array_6.push_back(drone_out_3);
        }
      }

      // Check link 4-5
      if(getDistance(drone_out_4.allocation, drone_out_5.allocation) < TARGET_RADIUS){
        if(drone_out_5.y_size !=0){
          DKF_array_4.push_back(drone_out_5);
        }
        if(drone_out_4.y_size !=0){
          DKF_array_5.push_back(drone_out_4);
        }
      }

      // Check link 4-6
      if(getDistance(drone_out_4.allocation, drone_out_6.allocation) < TARGET_RADIUS){
        if(drone_out_6.y_size !=0){
          DKF_array_4.push_back(drone_out_6);
        }
        if(drone_out_4.y_size !=0){
          DKF_array_6.push_back(drone_out_4);
        }
      }

      // Check link 5-6
      if(getDistance(drone_out_5.allocation, drone_out_6.allocation) < TARGET_RADIUS){
        if(drone_out_6.y_size !=0){
          DKF_array_5.push_back(drone_out_6);
        }
        if(drone_out_5.y_size !=0){
          DKF_array_6.push_back(drone_out_5);
        }
      }
      std::cout << "msg affect" <<std::endl;
      drone_input_1.array = DKF_array_1;
      drone_input_2.array = DKF_array_2;
      drone_input_3.array = DKF_array_3;
      drone_input_4.array = DKF_array_4;
      drone_input_5.array = DKF_array_5;
      drone_input_6.array = DKF_array_6;

      // Publish messages to drones od the swarm
      pub_output1.publish(drone_input_1);
      pub_output2.publish(drone_input_2);
      pub_output3.publish(drone_input_3);
      pub_output4.publish(drone_input_4);
      pub_output5.publish(drone_input_5);
      pub_output6.publish(drone_input_6);
    }
    std::cout << "loop" <<std::endl;
  
    ros::spinOnce();

    loop_rate.sleep();
  }
  return 0;
}