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

#define FALSE 0
#define TRUE 1
#define COM_RANGE 1.5
#define TARGET_RADIUS 1.5

//----------------------------------------------------------
//                       Global variables
//----------------------------------------------------------


int flag_callback_estimation = FALSE;
int flag_callback_innovation = FALSE;
int flag_callback_odom = FALSE;
int flag_callback_mod = FALSE;

geometry_msgs::PoseWithCovarianceStamped est_drone1;
geometry_msgs::PoseWithCovarianceStamped est_drone2;

geometry_msgs::Pose est_target_pose;

std::vector<geometry_msgs::Pose> est_array;

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

//----------------------------------------------------------
//                        Main
//----------------------------------------------------------

int main(int argc, char **argv){
  //----------------------------
  // Init
  ros::init(argc, argv, "evaluation_node");

  //----------------------------
  // Node handler with image transport
  ros::NodeHandle n;

  //----------------------------
  // Suscribers and publishers

  // DKF subscribers
  message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped> sub_estimate1(n, "/quadrotor1/estimate", 1);
  message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped> sub_estimate2(n, "/quadrotor0/estimate", 1);
  message_filters::Subscriber<geometry_msgs::PoseStamped> sub_target_est(n, "/target", 1);

  // Odometry subscribers
  message_filters::Subscriber<nav_msgs::Odometry> sub_odom1(n, "/quadrotor1/ground_truth/state", 1);
  message_filters::Subscriber<nav_msgs::Odometry> sub_odom2(n, "/quadrotor0/ground_truth/state", 1);
  message_filters::Subscriber<geometry_msgs::PoseStamped> sub_target_odom(n, "/target", 1);

  // Synchroniser policies
  typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseWithCovarianceStamped, geometry_msgs::PoseWithCovarianceStamped, geometry_msgs::PoseStamped> SyncPolicy;
  message_filters::Synchronizer<SyncPolicy> syncE(SyncPolicy(10), sub_estimate1, sub_estimate2, sub_target_est);
  syncE.registerCallback(boost::bind(&EstimateCallback, _1, _2, _3));

  typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, nav_msgs::Odometry, geometry_msgs::PoseStamped> OdomSyncPolicy;
  message_filters::Synchronizer<OdomSyncPolicy> syncO(OdomSyncPolicy(10), sub_odom1, sub_odom2, sub_target_odom);
  syncO.registerCallback(boost::bind(&OdomCallback, _1, _2, _3));

  // Neighbours message publishers
  ros::Publisher pub_mean_error = n.advertise<std_msgs::Float64>("mean_error",1000);
  ros::Publisher pub_error_1 = n.advertise<std_msgs::Float64>("error_1",1000);
  ros::Publisher pub_error_2 = n.advertise<std_msgs::Float64>("error_2",1000);


  ros::Publisher pub_minimal_distance = n.advertise<std_msgs::Float64>("minimal_distance",1000);
  ros::Publisher pub_target_distance_1 = n.advertise<std_msgs::Float64>("target_distance_1",1000);
  ros::Publisher pub_target_distance_2 = n.advertise<std_msgs::Float64>("target_distance_2",1000);


  ros::Publisher pub_mean_velocity = n.advertise<std_msgs::Float32MultiArray>("mean_velocity",1000);
  ros::Publisher pub_biggest_cluster = n.advertise<std_msgs::Int32>("biggest_cluster",1000);
  ros::Publisher pub_order = n.advertise<std_msgs::Float32MultiArray>("order",1000);

  

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

  double mean_error;
  double mean_velocity;
  int bigger_cluster;

  std::vector<double> distances_swarm;
  std::vector<float> mean_velocities;

  std::vector<float> orders;

  // Error messages
  std_msgs::Float64 mean_error_msg;
  std_msgs::Float64 error_1_msg;
  std_msgs::Float64 error_2_msg;

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

    if(flag_callback_estimation){
      // Classify drone in function of their allocated target
      subswarms_indexes.clear();
      int max_subswarm_size = 0;

      for(int i=0; i<targets_poses.size(); i++){
        int subswarm_size = 0;
        std::vector<int> subswarm_index;
        for(int j=0; j<est_array.size(); j++){
          if(getDistance(est_array[j], targets_poses[i]) < TARGET_RADIUS){
            subswarm_index.push_back(j);
            subswarm_size += 1;
          }
        }
        if(subswarm_size > max_subswarm_size){
          max_subswarm_size = subswarm_size;
        }
        subswarms_indexes.push_back(subswarm_index);
      }

      targets_indexes.clear();
      for(int i=0; i<est_array.size(); i++){
        for(int j=0; j<targets_poses.size(); j++){
          if(getDistance(est_array[i],targets_poses[j]) < TARGET_RADIUS){
            targets_indexes.push_back(j);
          }
        }
      }



      // Generate error messages
      error_drone1 = getDistance(est_target_pose, est_drone1.pose.pose);
      error_drone2 = getDistance(est_target_pose, est_drone2.pose.pose);

      mean_error = (error_drone1 + error_drone2)/2.0;
      mean_error_msg.data = mean_error;
      error_1_msg.data = error_drone1;
      error_2_msg.data = error_drone2;
      

      pub_mean_error.publish(mean_error_msg);
      pub_error_1.publish(error_1_msg);
      pub_error_2.publish(error_2_msg);
    }


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
    

    ros::spinOnce();

    loop_rate.sleep();
  }
  return 0;
}