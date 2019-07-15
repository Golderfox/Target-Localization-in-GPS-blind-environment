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
#define TARGET_RADIUS 1.0

//----------------------------------------------------------
//                       Global variables
//----------------------------------------------------------


int flag_callback = FALSE;
int flag_callback_drone = FALSE;
int flag_callback_mod = FALSE;
gazebo_msgs::ModelStates gt_ptr;
nav_msgs::Odometry odom_ptr;
std_msgs::Header img_header;

std::vector<geometry_msgs::Pose> sensor_poses_1; 
std::vector<geometry_msgs::Pose> sensor_poses_2; 
std::vector<geometry_msgs::Pose> sensor_poses_3;
std::vector<geometry_msgs::Pose> sensor_poses_4; 
std::vector<geometry_msgs::Pose> sensor_poses_5; 
std::vector<geometry_msgs::Pose> sensor_poses_6;

std::vector<std::vector<geometry_msgs::Pose> > sensors_array;

std::vector<geometry_msgs::Pose> sent_poses_1; 
std::vector<geometry_msgs::Pose> sent_poses_2; 
std::vector<geometry_msgs::Pose> sent_poses_3;
std::vector<geometry_msgs::Pose> sent_poses_4; 
std::vector<geometry_msgs::Pose> sent_poses_5; 
std::vector<geometry_msgs::Pose> sent_poses_6;

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
void Callback(const geometry_msgs::PoseArray::ConstPtr msg_sensor_1, const geometry_msgs::PoseArray::ConstPtr msg_sensor_2, const geometry_msgs::PoseArray::ConstPtr msg_sensor_3, const geometry_msgs::PoseArray::ConstPtr msg_sensor_4, const geometry_msgs::PoseArray::ConstPtr msg_sensor_5, const geometry_msgs::PoseArray::ConstPtr msg_sensor_6)
{
  sensor_poses_1.clear();
  sensor_poses_2.clear();
  sensor_poses_3.clear();
  sensor_poses_4.clear();
  sensor_poses_5.clear();
  sensor_poses_6.clear();

  sensors_array.clear();


  ROS_INFO("Sensors");
  for(int i=0; i<msg_sensor_1->poses.size(); i++){
    sensor_poses_1.push_back(msg_sensor_1->poses[i]);
  }
  for(int i=0; i<msg_sensor_2->poses.size(); i++){
    sensor_poses_2.push_back(msg_sensor_2->poses[i]);
  }
  for(int i=0; i<msg_sensor_3->poses.size(); i++){
    sensor_poses_3.push_back(msg_sensor_3->poses[i]);
  }
  for(int i=0; i<msg_sensor_4->poses.size(); i++){
    sensor_poses_4.push_back(msg_sensor_4->poses[i]);
  }
  for(int i=0; i<msg_sensor_5->poses.size(); i++){
    sensor_poses_5.push_back(msg_sensor_5->poses[i]);
  }
  for(int i=0; i<msg_sensor_6->poses.size(); i++){
    sensor_poses_6.push_back(msg_sensor_6->poses[i]);
  }
  sensors_array.push_back(sensor_poses_1);
  sensors_array.push_back(sensor_poses_2);
  sensors_array.push_back(sensor_poses_3);
  sensors_array.push_back(sensor_poses_4);
  sensors_array.push_back(sensor_poses_5);
  sensors_array.push_back(sensor_poses_6);
  header = msg_sensor_1->header;
  flag_callback = TRUE;
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
  
  message_filters::Subscriber<geometry_msgs::PoseArray> sub_sensor_1(n, "/quadrotor1/targets_poses", 1);
  message_filters::Subscriber<geometry_msgs::PoseArray> sub_sensor_2(n, "/quadrotor2/targets_poses", 1);
  message_filters::Subscriber<geometry_msgs::PoseArray> sub_sensor_3(n, "/quadrotor3/targets_poses", 1);
  message_filters::Subscriber<geometry_msgs::PoseArray> sub_sensor_4(n, "/quadrotor4/targets_poses", 1);
  message_filters::Subscriber<geometry_msgs::PoseArray> sub_sensor_5(n, "/quadrotor5/targets_poses", 1);
  message_filters::Subscriber<geometry_msgs::PoseArray> sub_sensor_6(n, "/quadrotor6/targets_poses", 1);

  typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseArray, geometry_msgs::PoseArray, geometry_msgs::PoseArray, geometry_msgs::PoseArray, geometry_msgs::PoseArray, geometry_msgs::PoseArray> SyncPolicy;
  message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(10), sub_sensor_1, sub_sensor_2, sub_sensor_3, sub_sensor_4, sub_sensor_5, sub_sensor_6);
  sync.registerCallback(boost::bind(&Callback, _1, _2, _3, _4, _5, _6));

  ros::Publisher pub_drone_1 = n.advertise<geometry_msgs::PoseArray>("/quadrotor1/neighbours_sensors",1000);
  ros::Publisher pub_drone_2 = n.advertise<geometry_msgs::PoseArray>("/quadrotor2/neighbours_sensors",1000);
  ros::Publisher pub_drone_3 = n.advertise<geometry_msgs::PoseArray>("/quadrotor3/neighbours_sensors",1000);
  ros::Publisher pub_drone_4 = n.advertise<geometry_msgs::PoseArray>("/quadrotor4/neighbours_sensors",1000);
  ros::Publisher pub_drone_5 = n.advertise<geometry_msgs::PoseArray>("/quadrotor5/neighbours_sensors",1000);
  ros::Publisher pub_drone_6 = n.advertise<geometry_msgs::PoseArray>("/quadrotor6/neighbours_sensors",1000);
  //----------------------------
  // Rate
  ros::Rate loop_rate(30);

  //----------------------------
  // Init messages
  geometry_msgs::PoseArray targets_msg;

  std::vector<geometry_msgs::Pose> targets_array;

  //----------------------------
  // While loop
  while (ros::ok())
  {
    targets_array.clear();
    if(flag_callback){
      for(int i=0; i<sensors_array.size(); i++){
        if(sensors_array[i].size()>0){
          targets_array.push_back(sensors_array[i][0]);
          break;
        }
      }

      for(int i=0; i<sensors_array.size(); i++){
        for(int j=0; j<sensors_array[i].size(); j++){
          for(int k=0; k<targets_array.size(); k++){
            if(getDistance(sensors_array[i][j],targets_array[k])> TARGET_RADIUS){
              targets_array.push_back(sensors_array[i][j]);
            }
          }
        }
      }

      // Assign message
      targets_msg.poses = targets_array;
      targets_msg.header = header;
      pub_drone_1.publish(targets_msg);
      pub_drone_2.publish(targets_msg);
      pub_drone_3.publish(targets_msg);
      pub_drone_4.publish(targets_msg);
      pub_drone_5.publish(targets_msg);
      pub_drone_6.publish(targets_msg);
      
    }
    
  
    
    ros::spinOnce();

    loop_rate.sleep();
  }
  return 0;
}
