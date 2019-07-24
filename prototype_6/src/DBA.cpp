#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Header.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Int32.h"

#include <nav_msgs/Odometry.h>
#include "tf/transform_datatypes.h"

#include "stdlib.h"
#include "stdio.h"
#include <string.h>
#include <cmath>
#include <random_numbers/random_numbers.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>


#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <Eigen3/Eigen/Dense>

#include <aruco_msgs/MarkerArray.h>


#define FALSE 0
#define TRUE 1
#define COM_RANGE 0.5
#define CONTROL_ENABLED 1

#define ALPHA 1
#define BETA 1

#define COEFF_P_YAW 3.0
#define COEFF_P_X 0.5
#define COEFF_P_Z 0.5
#define FLIGHT_HEIGHT 1.0

//----------------------------------------------------------
//                       Global variables
//----------------------------------------------------------


int flag_callback = FALSE;
int flag_callback_drone = FALSE;
int flag_callback_sensor = FALSE;
int flag_callback_swarm = FALSE;
int flag_callback_com_in = FALSE;
int flag_callback_odom = FALSE;
int flag_callback_targets = FALSE;
int flag_target_assigned = FALSE;

std::vector<geometry_msgs::Pose> targets_poses;
nav_msgs::Odometry odom_ptr;

std::vector<geometry_msgs::Pose> neighbours;

std_msgs::Header header;



//----------------------------------------------------------
//                    Defining functions
//----------------------------------------------------------

// --> return the distance between two drones
float getDistance(geometry_msgs::Pose drone_pose1, geometry_msgs::Pose drone_pose2){
  return sqrt(std::pow((drone_pose1.position.x)-(drone_pose2.position.x),2.0) + std::pow((drone_pose1.position.y)-(drone_pose2.position.y),2.0) + std::pow((drone_pose1.position.z)-(drone_pose2.position.z),2.0));
}

// --> Convert an Eigen matrix into a vector of doubles
std::vector<double> matrixToMsg(Eigen::MatrixXd mat){
  std::vector<double> vect;
  for(int i=0; i<mat.size(); i++){
    vect.push_back(*(mat.data()+i));
  }
  return vect;
}

// --> Convert an Eigen matrix into a boost array of doubles
boost::array<double,36> matrixToCovMsg(Eigen::MatrixXd mat){
  boost::array<double,36> cov;
  for(int i=0; i<mat.size(); i++){
    cov[i] = *(mat.data()+i);
  }
  return cov;
}


// --> Convert a vector of doubles into a matrix (Eigen matrix)
Eigen::MatrixXd msgToMatrix(std::vector<double> msg, int mat_size){
  Eigen::MatrixXd mat(mat_size, mat_size);
  for(int i=0; i<msg.size(); i++){
    *(mat.data()+i) = msg[i];
  }
  return mat;
}

// --> Convert a vector of doubles into a vector (Eigen matrix)
Eigen::MatrixXd msgToVEctor(std::vector<double> msg, int vec_size){
  Eigen::MatrixXd vec(vec_size, 1);
  for(int i=0; i<msg.size(); i++){
    *(vec.data()+i) = msg[i];
  }
  return vec;
}
/*
Eigen::Vector2d repulsion(geometry_msgs::Pose p, geometry_msgs::Pose q){
  Eigen::Vector2d repusion = Eigen::Vector2d::Zero(2,1);
  Eigen::Vector2d p_vect = Eigen::Vector2d::Zero(2,1);
  p_vect(0,0) = p.position.x;
  p_vect(1,0) = p.position.y;
  Eigen::Vector2d q_vect = Eigen::Vector2d::Zero(2,1);
  q_vect(0,0) = q.position.x;
  q_vect(1,0) = q.position.y;

  repulsion = (p_vect - q_vect)/(std::pow((p_vect - q_vect).norm(),3.0));
  
  return repulsion;
}
*/
//----------------------------------------------------------
//                       DKFCallback
//----------------------------------------------------------

void targetsCallback(const aruco_msgs::MarkerArray& msg)
{

  header = msg.header;
  targets_poses.clear();
  ROS_INFO("targets_markers");
  for(int i=0; i<msg.markers.size();i++)
  {
    targets_poses.push_back(msg.markers[i].pose.pose);
  }
  flag_callback_targets = TRUE;
}

void odomCallback(const nav_msgs::Odometry& msg)
{
  ROS_INFO("Odometry");
  odom_ptr = msg;
  flag_callback_odom = TRUE;
}
/*
void clearedCallback(const nav_msgs::Odometry& msg)
{
  ROS_INFO("Odometry");
  odom_ptr = msg;
  flag_callback_odom = TRUE;
}
*/



//----------------------------------------------------------
//                        Main
//----------------------------------------------------------

int main(int argc, char **argv){
  //----------------------------
  // Init
  std::cout << "init" << std::endl;
  ros::init(argc, argv, "DBA");
  std::cout << "init done" << std::endl;

  //----------------------------
  // Node handler with image transport
  ros::NodeHandle n;

  //----------------------------
  // Suscribers and publishers

  // Odometry subscribers
  /*
  message_filters::Subscriber<prototype_3::DKF> sub_input1(n, "/quadrotor1/ground_truth/state", 1);
  message_filters::Subscriber<prototype_3::DKF> sub_input2(n, "/quadrotor2/ground_truth/state", 1);
  message_filters::Subscriber<prototype_3::DKF> sub_input3(n, "/quadrotor3/ground_truth/state", 1);
  typedef message_filters::sync_policies::ApproximateTime<prototype_3::DKF, prototype_3::DKF, prototype_3::DKF> SyncPolicy;
  message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(10), sub_input1, sub_input2, sub_input3);
  sync.registerCallback(boost::bind(&Callback, _1, _2, _3));
  */
  //ros::Subscriber sub_cleared = n.subscribe("ground_truth/state", 1, clearedCallback);
  ros::Subscriber sub_odom = n.subscribe("ground_truth/state", 1, odomCallback);
  ros::Subscriber sub_poses = n.subscribe("neighbours_sensors", 1, targetsCallback);
  /*
  // Neighbours message subscribers
  ros::Subscriber sub_input1 = n.subscribe("/quadrotor1/DKF_out", 1, drone1Callback);
  ros::Subscriber sub_input2 = n.subscribe("/quadrotor2/DKF_out", 1, drone2Callback);
  ros::Subscriber sub_input3 = n.subscribe("/quadrotor3/DKF_out", 1, drone3Callback);
  */

  // Neighbours message publishers
  ros::Publisher pub_target_affect = n.advertise<std_msgs::Int32>("target_affectation",1000);
  //ros::Publisher pub_distribution = n.advertise<std_msgs::Float64MultiArray>("target_distribution",1000);
  ros::Publisher pub_sensor_pose = n.advertise<geometry_msgs::PoseStamped>("target_affectation_pose",1000);

  std_msgs::Header header;
  

  //----------------------------
  // Set msgs


  geometry_msgs::Twist command;
  std_msgs::Int32 target_affect;
  std_msgs::Float64MultiArray target_distribution_msg;
  std_msgs::Float64MultiArray target_costs_msg;
  geometry_msgs::PoseStamped target_pose_msg;


  std::vector<double> costs(3,1.0);//{0.01, 0.01, 0.01};
  std::vector<double> qualities(3,1.0/3.0);
  //qualities[0] = 1.0/7.0; qualities[1] = 2.0/7.0; qualities[2] = 4.0/7.0;
  qualities[0] = 1.0/3.0; qualities[1] = 1.0/3.0; qualities[2] = 1.0/3.0;
  std::vector<double> utilities;//{0.0, 0.0, 0.0};
  double sum_cq = 0.0;
  double decision = 0.0;

  random_numbers::RandomNumberGenerator generator;




  //----------------------------
  // Rate
  ros::Rate loop_rate(30);

  //----------------------------
  // While loop
  while (ros::ok()){
    std::cout << "loop" << std::endl;
    // Header
    header.frame_id = "nav";
    header.stamp = ros::Time::now();

    if((flag_callback_targets && flag_callback_odom) && !flag_target_assigned){
      //std::cout << "DBA" << std::endl;
      sum_cq = 0.0;
      costs.clear();
      utilities.clear();
      //std::cout << "init DBA" << std::endl;
      std::cout << targets_poses.size() << std::endl;
      ROS_INFO("cost");
      for(int i=0; i<targets_poses.size(); i++){
        costs.push_back((getDistance(odom_ptr.pose.pose, targets_poses[i])));
        ROS_INFO("cost_loop: %f", costs[i]);
        sum_cq += std::pow(qualities[i],ALPHA) * std::pow(1/costs[i],BETA);
      }
      ROS_INFO("util");
      for(int i=0; i<targets_poses.size(); i++){
        utilities.push_back((std::pow(qualities[i],ALPHA) * std::pow(1/costs[i],BETA))/sum_cq);
      }
      target_costs_msg.data = costs;
      target_distribution_msg.data = utilities;
      decision = generator.uniformReal(0.0, 1.0);
      ROS_INFO("decis");
      for(int i=0; i<targets_poses.size(); i++){
        decision -= utilities[i];
        if(decision < 0.0){
          target_affect.data = i;
          flag_target_assigned = TRUE;
          break;
        }
      }
      std::cout << "end DBA" << std::endl;

      
    }

    if(flag_target_assigned){
      std::cout << "pub DBA" << std::endl;
      target_pose_msg.pose = targets_poses[target_affect.data];
      target_pose_msg.header = header;
      pub_target_affect.publish(target_affect);
      pub_sensor_pose.publish(target_pose_msg);
      //pub_costs.publish(target_costs_msg);
      //pub_distribution.publish(target_distribution_msg);
      std::cout << "pub DBA done" << std::endl;
    }

    ros::spinOnce();

    loop_rate.sleep();
  }
  return 0;
}