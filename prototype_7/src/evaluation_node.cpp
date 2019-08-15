#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float64.h"
#include <std_msgs/Float32MultiArray.h>
#include "std_msgs/Header.h"

#include <nav_msgs/Odometry.h>


#include "stdlib.h"
#include "stdio.h"
#include <string.h>
#include <cmath>
#include <complex>
#include "tf/transform_datatypes.h"

#include <prototype_3/DKF.h>
#include <prototype_3/DKFStamped.h>
#include <prototype_3/DKF_multi.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/Pose.h>

#include <aruco_msgs/MarkerArray.h>

#define FALSE 0
#define TRUE 1
#define COM_RANGE 1.5
#define TARGET_RADIUS 1.5

//----------------------------------------------------------
//                       Global variables
//----------------------------------------------------------


int flag_callback_estimation = FALSE;
int flag_callback_sensor = FALSE;
int flag_callback_innovation = FALSE;
int flag_callback_odom = FALSE;
int flag_callback_mod = FALSE;
int flag_callback_tag = FALSE;

geometry_msgs::PoseWithCovarianceStamped est_drone1;
geometry_msgs::PoseWithCovarianceStamped est_drone2;

geometry_msgs::PoseWithCovarianceStamped sensor_drone1;
geometry_msgs::PoseWithCovarianceStamped sensor_drone2;

geometry_msgs::Pose est_target_pose;
geometry_msgs::Pose tag_pose;

std::vector<geometry_msgs::Pose> est_array;

std::vector<geometry_msgs::Pose> sensor_array;

geometry_msgs::PoseWithCovarianceStamped innov_drone;

nav_msgs::Odometry odom_ptr1;
nav_msgs::Odometry odom_ptr2;

geometry_msgs::Pose odom_target_pose;

std::vector<nav_msgs::Odometry> odom_array;


//----------------------------------------------------------
//                    Defining functions
//----------------------------------------------------------

// --> return the distance between two drones
double getDistance(geometry_msgs::Pose drone_pose1, geometry_msgs::Pose drone_pose2){
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

void EstimateCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr est_msg_1, const geometry_msgs::PoseWithCovarianceStamped::ConstPtr est_msg_2, const geometry_msgs::PoseStamped::ConstPtr target_msg){
  ROS_INFO("Estimation");
  est_array.clear();
  flag_callback_estimation = TRUE;
  est_drone1 = *est_msg_1;
  est_drone2 = *est_msg_2;
  est_target_pose = target_msg->pose;

  est_array.push_back(est_msg_1->pose.pose);
  est_array.push_back(est_msg_2->pose.pose);

}

void EstimateCallbackBis(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr est_msg_1, const geometry_msgs::PoseWithCovarianceStamped::ConstPtr est_msg_2, const geometry_msgs::TransformStamped::ConstPtr target_msg){
  ROS_INFO("Estimation");
  est_array.clear();
  flag_callback_estimation = TRUE;
  est_drone1 = *est_msg_1;
  est_drone2 = *est_msg_2;
  est_target_pose.position.x = target_msg->transform.translation.x;
  est_target_pose.position.y = target_msg->transform.translation.y;
  est_target_pose.position.z = target_msg->transform.translation.z;

  est_target_pose.orientation.x = target_msg->transform.rotation.x;
  est_target_pose.orientation.y = target_msg->transform.rotation.y;
  est_target_pose.orientation.z = target_msg->transform.rotation.z;
  est_target_pose.orientation.w = target_msg->transform.rotation.w;


  est_array.push_back(est_msg_1->pose.pose);
  est_array.push_back(est_msg_2->pose.pose);

}

void EstimateCallbackTer(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr est_msg_1, const geometry_msgs::PoseWithCovarianceStamped::ConstPtr est_msg_2){
  ROS_INFO("Estimation");
  est_array.clear();
  flag_callback_estimation = TRUE;
  est_drone1 = *est_msg_1;
  est_drone2 = *est_msg_2;
  est_array.push_back(est_msg_1->pose.pose);
  est_array.push_back(est_msg_2->pose.pose);

}

void SensorCallback(const aruco_msgs::MarkerArray::ConstPtr sen_msg_1, const aruco_msgs::MarkerArray::ConstPtr sen_msg_2, const geometry_msgs::TransformStamped::ConstPtr target_msg){
  ROS_INFO("Estimation");
  est_array.clear();
  flag_callback_sensor = TRUE;
  sensor_drone1.header = sen_msg_1->header;
  sensor_drone2.header = sen_msg_2->header;
  sensor_drone1.pose = sen_msg_1->markers[0].pose;
  sensor_drone2.pose = sen_msg_2->markers[0].pose;
  est_target_pose.position.x = target_msg->transform.translation.x;
  est_target_pose.position.y = target_msg->transform.translation.y;
  est_target_pose.position.z = target_msg->transform.translation.z;

  est_target_pose.orientation.x = target_msg->transform.rotation.x;
  est_target_pose.orientation.y = target_msg->transform.rotation.y;
  est_target_pose.orientation.z = target_msg->transform.rotation.z;
  est_target_pose.orientation.w = target_msg->transform.rotation.w;


  sensor_array.push_back(sensor_drone1.pose.pose);
  sensor_array.push_back(sensor_drone2.pose.pose);

}

void SensorCallbackBis(const aruco_msgs::MarkerArray::ConstPtr sen_msg_1, const aruco_msgs::MarkerArray::ConstPtr sen_msg_2){
  ROS_INFO("SensorBis");
  sensor_array.clear();
  flag_callback_sensor = TRUE;
  sensor_drone1.header = sen_msg_1->header;
  sensor_drone2.header = sen_msg_2->header;
  sensor_drone1.pose = sen_msg_1->markers[0].pose;
  sensor_drone2.pose = sen_msg_2->markers[0].pose;

  sensor_array.push_back(sensor_drone1.pose.pose);
  sensor_array.push_back(sensor_drone2.pose.pose);

}

void OdomCallback(const nav_msgs::Odometry::ConstPtr odom_msg_1, const nav_msgs::Odometry::ConstPtr odom_msg_2, const geometry_msgs::PoseStamped::ConstPtr target_msg){
  ROS_INFO("Odom");
  odom_array.clear();
  flag_callback_odom = TRUE;
  odom_ptr1 = *odom_msg_1;
  odom_ptr2 = *odom_msg_2;
  odom_target_pose = target_msg->pose;

  odom_array.push_back(*odom_msg_1);
  odom_array.push_back(*odom_msg_2);

}

void tagCallback(const geometry_msgs::TransformStamped& msg){
  ROS_INFO("Tag");
  flag_callback_tag = TRUE;
  tag_pose.position.x = msg.transform.translation.x;
  tag_pose.position.y = msg.transform.translation.y;
  tag_pose.position.z = msg.transform.translation.z;

  tag_pose.orientation.x = msg.transform.rotation.x;
  tag_pose.orientation.y = msg.transform.rotation.y;
  tag_pose.orientation.z = msg.transform.rotation.z;
  tag_pose.orientation.w = msg.transform.rotation.w;

}

//----------------------------------------------------------
//                        Main
//----------------------------------------------------------

int main(int argc, char **argv){
  //----------------------------
  // Init
  std::cout << "init" << std::endl;
  ros::init(argc, argv, "evaluation_node");

  //----------------------------
  // Node handler with image transport
  ros::NodeHandle n;

  //----------------------------
  // Suscribers and publishers

  // DKF subscribers
  message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped> sub_estimate1(n, "/quadrotor1/estimate", 1);
  message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped> sub_estimate2(n, "/quadrotor0/estimate", 1);
  //message_filters::Subscriber<geometry_msgs::PoseStamped> sub_target_est(n, "/target", 1);
  //message_filters::Subscriber<geometry_msgs::TransformStamped> sub_target_est_bis(n, "/vicon/tag/tag", 1);

  message_filters::Subscriber<aruco_msgs::MarkerArray> sub_sensor1(n, "/quadrotor1/aruco_marker_publisher/markers", 1);
  message_filters::Subscriber<aruco_msgs::MarkerArray> sub_sensor2(n, "/quadrotor0/aruco_marker_publisher/markers", 1);
  /*
  // Odometry subscribers
  message_filters::Subscriber<nav_msgs::Odometry> sub_odom1(n, "/quadrotor1/ground_truth/state", 1);
  message_filters::Subscriber<nav_msgs::Odometry> sub_odom2(n, "/quadrotor0/ground_truth/state", 1);
  message_filters::Subscriber<geometry_msgs::PoseStamped> sub_target_odom(n, "/target", 1);
  */
  ros::Subscriber sub_tag = n.subscribe("/vicon/tag/tag", 1, tagCallback);

  /*
  // Synchroniser policies
  typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseWithCovarianceStamped, geometry_msgs::PoseWithCovarianceStamped, geometry_msgs::TransformStamped> SyncPolicy;
  message_filters::Synchronizer<SyncPolicy> syncE(SyncPolicy(10), sub_estimate1, sub_estimate2, sub_target_est_bis);
  syncE.registerCallback(boost::bind(&EstimateCallbackBis, _1, _2, _3));
  */

   
  // Synchroniser policies
  typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseWithCovarianceStamped, geometry_msgs::PoseWithCovarianceStamped> SyncPolicy;
  message_filters::Synchronizer<SyncPolicy> syncE(SyncPolicy(10), sub_estimate1, sub_estimate2);
  syncE.registerCallback(boost::bind(&EstimateCallbackTer, _1, _2));
  


  /*
  // Synchroniser policies
  typedef message_filters::sync_policies::ApproximateTime<aruco_msgs::MarkerArray, aruco_msgs::MarkerArray, geometry_msgs::TransformStamped> SensorSyncPolicy;
  message_filters::Synchronizer<SensorSyncPolicy> syncS(SensorSyncPolicy(10), sub_sensor1, sub_sensor2, sub_target_est_bis);
  syncS.registerCallback(boost::bind(&SensorCallback, _1, _2, _3));
  */


  // Synchroniser policies
  
  typedef message_filters::sync_policies::ApproximateTime<aruco_msgs::MarkerArray, aruco_msgs::MarkerArray> SensorSyncPolicy;
  message_filters::Synchronizer<SensorSyncPolicy> syncS(SensorSyncPolicy(10), sub_sensor1, sub_sensor2);
  syncS.registerCallback(boost::bind(&SensorCallbackBis, _1, _2));
  
  /*  
  typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, nav_msgs::Odometry, geometry_msgs::PoseStamped> OdomSyncPolicy;
  message_filters::Synchronizer<OdomSyncPolicy> syncO(OdomSyncPolicy(10), sub_odom1, sub_odom2, sub_target_odom);
  syncO.registerCallback(boost::bind(&OdomCallback, _1, _2, _3));
  */
  // Neighbours message publishers
  ros::Publisher pub_mean_error = n.advertise<std_msgs::Float64>("mean_error",1000);
  ros::Publisher pub_mean_sensor_error = n.advertise<std_msgs::Float64>("mean_sensor_error",1000);
  ros::Publisher pub_error_1 = n.advertise<std_msgs::Float64>("error_1",1000);
  ros::Publisher pub_error_2 = n.advertise<std_msgs::Float64>("error_2",1000);
  ros::Publisher pub_sensor_error_1 = n.advertise<std_msgs::Float64>("sensor_error_1",1000);
  ros::Publisher pub_sensor_error_2 = n.advertise<std_msgs::Float64>("sensor_error_2",1000);

  /*
  ros::Publisher pub_minimal_distance = n.advertise<std_msgs::Float64>("minimal_distance",1000);
  ros::Publisher pub_target_distance_1 = n.advertise<std_msgs::Float64>("target_distance_1",1000);
  ros::Publisher pub_target_distance_2 = n.advertise<std_msgs::Float64>("target_distance_2",1000);
  */

  /*
  ros::Publisher pub_mean_velocity = n.advertise<std_msgs::Float32MultiArray>("mean_velocity",1000);
  ros::Publisher pub_biggest_cluster = n.advertise<std_msgs::Int32>("biggest_cluster",1000);
  ros::Publisher pub_order = n.advertise<std_msgs::Float32MultiArray>("order",1000);
  */

  

  //----------------------------
  // Init targets
  std::vector< std::vector<int> > subswarms_indexes;
  std::vector<int> targets_indexes;
  geometry_msgs::Pose target_pose_1; geometry_msgs::Pose target_pose_2; geometry_msgs::Pose target_pose_3;
  std::vector<geometry_msgs::Pose> targets_poses;
  // Target 1
  target_pose_1.position.x = 5.0; target_pose_1.position.y = 0.0; target_pose_1.position.z = 0.5;
  target_pose_1.orientation.x = 0.0; target_pose_1.orientation.y = 0.0; target_pose_1.orientation.z = 0.0; target_pose_1.orientation.w = 0.0;
  targets_poses.push_back(target_pose_1);


  //----------------------------
  // Init messages

  // Init variables
  double error_drone1;
  double error_drone2;

  double error_sensor1;
  double error_sensor2;

  double mean_error;
  double mean_sensor_error;
  double mean_velocity;
  int bigger_cluster;

  std::vector<double> distances_swarm;
  std::vector<float> mean_velocities;

  std::vector<float> orders;

  // Error messages
  std_msgs::Float64 mean_error_msg;
  std_msgs::Float64 error_1_msg;
  std_msgs::Float64 error_2_msg;
  std_msgs::Float64 sensor_mean_error_msg;

  // Flocking messages
  std_msgs::Float64 minimal_distance;
  std_msgs::Float64 target_distance_1;
  std_msgs::Float64 target_distance_2;

  std_msgs::Float32MultiArray order_msg;
  
  //----------------------------
  // Rate
  ros::Rate loop_rate(30);

  //----------------------------
  // While loop
  while (ros::ok()){
    
    if(flag_callback_estimation && flag_callback_tag){

      // Generate error messages
      error_drone1 = getDistance(tag_pose, est_drone1.pose.pose);
      error_drone2 = getDistance(tag_pose, est_drone2.pose.pose);

      mean_error = (error_drone1 + error_drone2)/2.0;
      mean_error_msg.data = mean_error;
      error_1_msg.data = error_drone1;
      error_2_msg.data = error_drone2;
      

      pub_mean_error.publish(mean_error_msg);
      pub_error_1.publish(error_1_msg);
      pub_error_2.publish(error_2_msg);
    }

    if(flag_callback_sensor && flag_callback_tag){
      error_sensor1 = getDistance(tag_pose, sensor_drone1.pose.pose);
      error_sensor2 = getDistance(tag_pose, sensor_drone2.pose.pose);
      

      mean_sensor_error = (error_sensor1 + error_sensor2)/2.0;
      ROS_INFO("value: %f", mean_sensor_error);
      sensor_mean_error_msg.data = mean_sensor_error;
      pub_mean_sensor_error.publish(sensor_mean_error_msg);
    }

      /*
      if(flag_callback_odom){
        // Compute minimal distance within the swarm
        distances_swarm.clear();
        distances_swarm.push_back(getDistance(odom_ptr1.pose.pose,odom_ptr2.pose.pose));


        minimal_distance.data = *std::min_element(distances_swarm.begin(), distances_swarm.end());
        
        target_distance_1.data = getDistance(odom_ptr1.pose.pose,odom_target_pose);
        target_distance_2.data = getDistance(odom_ptr2.pose.pose,odom_target_pose);
        
        
        orders.clear();
        std::complex<double> arg_sum(0,0);
        for(int i=0; i<subswarms_indexes.size(); i++){
          arg_sum = std::complex<double>(0.0,0.0);
          for(int j=0; j<subswarms_indexes[i].size(); j++){
            //std::complex<double> exp_arg = std::polar(1.0, odom_array[subswarms_indexes[i][j]]);
            
            // Create  tf quaternion from pose
            tf::Quaternion q(odom_array[subswarms_indexes[i][j]].pose.pose.orientation.x,odom_array[subswarms_indexes[i][j]].pose.pose.orientation.y, odom_array[subswarms_indexes[i][j]].pose.pose.orientation.z, odom_array[subswarms_indexes[i][j]].pose.pose.orientation.w);
            // Convert quaternion to RPY
            double yaw;
            double pitch;
            double roll;
            tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
            //ROS_INFO("yaw %f", yaw);
            arg_sum += std::polar<double>(1.0, yaw);
            //ROS_INFO("complex(%f, %f)", std::real(std::polar<double>(1.0, yaw)), std::imag(std::polar<double>(1.0, yaw)));
          }
          float order = 1/(float)(subswarms_indexes[i].size())*std::abs(arg_sum);
          //ROS_INFO("coeff %f", 1/(float)(subswarms_indexes[i].size()));
          //ROS_INFO("arg sum %f", std::abs(arg_sum));
          //ROS_INFO("complex_arg(%f, %f)", std::real(arg_sum), std::imag(arg_sum));
          //ROS_INFO("test order %f", order);
          //ROS_INFO("------------------------------------------");
          orders.push_back(order);
        }

        //mean_velocity = (odom_array[0].twist.twist.linear.x + odom_array[1].twist.twist.linear.x + odom_array[2].twist.twist.linear.x + odom_array[3].twist.twist.linear.x + odom_array[4].twist.twist.linear.x + odom_array[5].twist.twist.linear.x)/6.0;
        order_msg.data = orders;
        
        

        // Publish messages
        pub_minimal_distance.publish(minimal_distance);
        pub_target_distance_1.publish(target_distance_1);
        pub_target_distance_2.publish(target_distance_2);

        pub_order.publish(order_msg);

        
      }
      */
    

    ros::spinOnce();

    loop_rate.sleep();
  }
  return 0;
}