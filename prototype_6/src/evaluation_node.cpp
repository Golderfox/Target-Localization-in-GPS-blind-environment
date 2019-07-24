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
int flag_callback_allocation = FALSE;
int flag_callback_odom = FALSE;
int flag_callback_mod = FALSE;

geometry_msgs::PoseWithCovarianceStamped est_drone1;
geometry_msgs::PoseWithCovarianceStamped est_drone2;
geometry_msgs::PoseWithCovarianceStamped est_drone3;
geometry_msgs::PoseWithCovarianceStamped est_drone4;
geometry_msgs::PoseWithCovarianceStamped est_drone5;
geometry_msgs::PoseWithCovarianceStamped est_drone6;

std::vector<geometry_msgs::Pose> est_array;

geometry_msgs::PoseStamped alloc_drone1;
geometry_msgs::PoseStamped alloc_drone2;
geometry_msgs::PoseStamped alloc_drone3;
geometry_msgs::PoseStamped alloc_drone4;
geometry_msgs::PoseStamped alloc_drone5;
geometry_msgs::PoseStamped alloc_drone6;

std::vector<geometry_msgs::Pose> alloc_array;

geometry_msgs::PoseWithCovarianceStamped innov_drone;

nav_msgs::Odometry odom_ptr1;
nav_msgs::Odometry odom_ptr2;
nav_msgs::Odometry odom_ptr3;
nav_msgs::Odometry odom_ptr4;
nav_msgs::Odometry odom_ptr5;
nav_msgs::Odometry odom_ptr6;

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

void EstimateCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr est_msg_1, const geometry_msgs::PoseWithCovarianceStamped::ConstPtr est_msg_2, const geometry_msgs::PoseWithCovarianceStamped::ConstPtr est_msg_3, const geometry_msgs::PoseWithCovarianceStamped::ConstPtr est_msg_4, const geometry_msgs::PoseWithCovarianceStamped::ConstPtr est_msg_5, const geometry_msgs::PoseWithCovarianceStamped::ConstPtr est_msg_6){
  ROS_INFO("Estimation");
  est_array.clear();
  flag_callback_estimation = TRUE;
  est_drone1 = *est_msg_1;
  est_drone2 = *est_msg_2;
  est_drone3 = *est_msg_3;
  est_drone4 = *est_msg_4;
  est_drone5 = *est_msg_5;
  est_drone6 = *est_msg_6;

  est_array.push_back(est_msg_1->pose.pose);
  est_array.push_back(est_msg_2->pose.pose);
  est_array.push_back(est_msg_3->pose.pose);
  est_array.push_back(est_msg_4->pose.pose);
  est_array.push_back(est_msg_5->pose.pose);
  est_array.push_back(est_msg_6->pose.pose);
}

void AllocationCallback(const geometry_msgs::PoseStamped::ConstPtr alloc_msg_1, const geometry_msgs::PoseStamped::ConstPtr alloc_msg_2, const geometry_msgs::PoseStamped::ConstPtr alloc_msg_3, const geometry_msgs::PoseStamped::ConstPtr alloc_msg_4, const geometry_msgs::PoseStamped::ConstPtr alloc_msg_5, const geometry_msgs::PoseStamped::ConstPtr alloc_msg_6){
  ROS_INFO("Allocation");
  alloc_array.clear();
  flag_callback_allocation = TRUE;
  alloc_drone1 = *alloc_msg_1;
  alloc_drone2 = *alloc_msg_2;
  alloc_drone3 = *alloc_msg_3;
  alloc_drone4 = *alloc_msg_4;
  alloc_drone5 = *alloc_msg_5;
  alloc_drone6 = *alloc_msg_6;

  alloc_array.push_back(alloc_msg_1->pose);
  alloc_array.push_back(alloc_msg_2->pose);
  alloc_array.push_back(alloc_msg_3->pose);
  alloc_array.push_back(alloc_msg_4->pose);
  alloc_array.push_back(alloc_msg_5->pose);
  alloc_array.push_back(alloc_msg_6->pose);
}

void OdomCallback(const nav_msgs::Odometry::ConstPtr odom_msg_1, const nav_msgs::Odometry::ConstPtr odom_msg_2, const nav_msgs::Odometry::ConstPtr odom_msg_3, const nav_msgs::Odometry::ConstPtr odom_msg_4, const nav_msgs::Odometry::ConstPtr odom_msg_5, const nav_msgs::Odometry::ConstPtr odom_msg_6){
  ROS_INFO("Odom");
  odom_array.clear();
  flag_callback_odom = TRUE;
  odom_ptr1 = *odom_msg_1;
  odom_ptr2 = *odom_msg_2;
  odom_ptr3 = *odom_msg_3;
  odom_ptr4 = *odom_msg_4;
  odom_ptr5 = *odom_msg_5;
  odom_ptr6 = *odom_msg_6;
  odom_array.push_back(*odom_msg_1);
  odom_array.push_back(*odom_msg_2);
  odom_array.push_back(*odom_msg_3);
  odom_array.push_back(*odom_msg_4);
  odom_array.push_back(*odom_msg_5);
  odom_array.push_back(*odom_msg_6);
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
  message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped> sub_estimate2(n, "/quadrotor2/estimate", 1);
  message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped> sub_estimate3(n, "/quadrotor3/estimate", 1);
  message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped> sub_estimate4(n, "/quadrotor4/estimate", 1);
  message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped> sub_estimate5(n, "/quadrotor5/estimate", 1);
  message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped> sub_estimate6(n, "/quadrotor6/estimate", 1);

  // Odometry subscribers
  message_filters::Subscriber<nav_msgs::Odometry> sub_odom1(n, "/quadrotor1/ground_truth/state", 1);
  message_filters::Subscriber<nav_msgs::Odometry> sub_odom2(n, "/quadrotor2/ground_truth/state", 1);
  message_filters::Subscriber<nav_msgs::Odometry> sub_odom3(n, "/quadrotor3/ground_truth/state", 1);
  message_filters::Subscriber<nav_msgs::Odometry> sub_odom4(n, "/quadrotor4/ground_truth/state", 1);
  message_filters::Subscriber<nav_msgs::Odometry> sub_odom5(n, "/quadrotor5/ground_truth/state", 1);
  message_filters::Subscriber<nav_msgs::Odometry> sub_odom6(n, "/quadrotor6/ground_truth/state", 1);

  // Allocation subscribers
  message_filters::Subscriber<geometry_msgs::PoseStamped> sub_alloc1(n, "/quadrotor1/target_affectation_pose", 1);
  message_filters::Subscriber<geometry_msgs::PoseStamped> sub_alloc2(n, "/quadrotor2/target_affectation_pose", 1);
  message_filters::Subscriber<geometry_msgs::PoseStamped> sub_alloc3(n, "/quadrotor3/target_affectation_pose", 1);
  message_filters::Subscriber<geometry_msgs::PoseStamped> sub_alloc4(n, "/quadrotor4/target_affectation_pose", 1);
  message_filters::Subscriber<geometry_msgs::PoseStamped> sub_alloc5(n, "/quadrotor5/target_affectation_pose", 1);
  message_filters::Subscriber<geometry_msgs::PoseStamped> sub_alloc6(n, "/quadrotor6/target_affectation_pose", 1);

  // Synchroniser policies
  typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseWithCovarianceStamped, geometry_msgs::PoseWithCovarianceStamped, geometry_msgs::PoseWithCovarianceStamped, geometry_msgs::PoseWithCovarianceStamped, geometry_msgs::PoseWithCovarianceStamped, geometry_msgs::PoseWithCovarianceStamped> SyncPolicy;
  message_filters::Synchronizer<SyncPolicy> syncE(SyncPolicy(10), sub_estimate1, sub_estimate2, sub_estimate3, sub_estimate4, sub_estimate5, sub_estimate6);
  syncE.registerCallback(boost::bind(&EstimateCallback, _1, _2, _3, _4, _5, _6));

  typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, nav_msgs::Odometry, nav_msgs::Odometry, nav_msgs::Odometry, nav_msgs::Odometry, nav_msgs::Odometry> OdomSyncPolicy;
  message_filters::Synchronizer<OdomSyncPolicy> syncO(OdomSyncPolicy(10), sub_odom1, sub_odom2, sub_odom3, sub_odom4, sub_odom5, sub_odom6);
  syncO.registerCallback(boost::bind(&OdomCallback, _1, _2, _3, _4, _5, _6));


  typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseStamped, geometry_msgs::PoseStamped, geometry_msgs::PoseStamped, geometry_msgs::PoseStamped, geometry_msgs::PoseStamped, geometry_msgs::PoseStamped> AllocSyncPolicy;
  message_filters::Synchronizer<AllocSyncPolicy> syncA(AllocSyncPolicy(10), sub_alloc1, sub_alloc2, sub_alloc3, sub_alloc4, sub_alloc5, sub_alloc6);
  syncA.registerCallback(boost::bind(&AllocationCallback, _1, _2, _3, _4, _5, _6));

  // Neighbours message publishers
  ros::Publisher pub_mean_error = n.advertise<std_msgs::Float64>("mean_error",1000);
  ros::Publisher pub_error_1 = n.advertise<std_msgs::Float64>("error_1",1000);
  ros::Publisher pub_error_2 = n.advertise<std_msgs::Float64>("error_2",1000);
  ros::Publisher pub_error_3 = n.advertise<std_msgs::Float64>("error_3",1000);
  ros::Publisher pub_error_4 = n.advertise<std_msgs::Float64>("error_4",1000);
  ros::Publisher pub_error_5 = n.advertise<std_msgs::Float64>("error_5",1000);
  ros::Publisher pub_error_6 = n.advertise<std_msgs::Float64>("error_6",1000);

  ros::Publisher pub_minimal_distance = n.advertise<std_msgs::Float64>("minimal_distance",1000);
  ros::Publisher pub_target_distance_1 = n.advertise<std_msgs::Float64>("target_distance_1",1000);
  ros::Publisher pub_target_distance_2 = n.advertise<std_msgs::Float64>("target_distance_2",1000);
  ros::Publisher pub_target_distance_3 = n.advertise<std_msgs::Float64>("target_distance_3",1000);
  ros::Publisher pub_target_distance_4 = n.advertise<std_msgs::Float64>("target_distance_4",1000);
  ros::Publisher pub_target_distance_5 = n.advertise<std_msgs::Float64>("target_distance_5",1000);
  ros::Publisher pub_target_distance_6 = n.advertise<std_msgs::Float64>("target_distance_6",1000);

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
  target_pose_1.position.x = 10.0; target_pose_1.position.y = 0.0; target_pose_1.position.z = 0.5;
  target_pose_1.orientation.x = 0.0; target_pose_1.orientation.y = 0.0; target_pose_1.orientation.z = 0.0; target_pose_1.orientation.w = 0.0;
  targets_poses.push_back(target_pose_1);

  // Target 2
  target_pose_2.position.x = 0.0; target_pose_2.position.y = -10.0; target_pose_2.position.z = 0.5;
  target_pose_2.orientation.x = 0.0; target_pose_2.orientation.y = 0.0; target_pose_2.orientation.z = 0.0; target_pose_2.orientation.w = 0.0;
  targets_poses.push_back(target_pose_2);

  // Target 3
  target_pose_3.position.x = 0.0; target_pose_3.position.y = 10.0; target_pose_3.position.z = 0.5;
  target_pose_3.orientation.x = 0.0; target_pose_3.orientation.y = 0.0; target_pose_3.orientation.z = 0.0; target_pose_3.orientation.w = 0.0;
  targets_poses.push_back(target_pose_3);

  //----------------------------
  // Init messages

  // Init variables
  double error_drone1;
  double error_drone2;
  double error_drone3;
  double error_drone4;
  double error_drone5;
  double error_drone6;
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
  std_msgs::Float64 error_3_msg;
  std_msgs::Float64 error_4_msg;
  std_msgs::Float64 error_5_msg;
  std_msgs::Float64 error_6_msg;

  // Flocking messages
  std_msgs::Float64 minimal_distance;
  std_msgs::Float64 target_distance_1;
  std_msgs::Float64 target_distance_2;
  std_msgs::Float64 target_distance_3;
  std_msgs::Float64 target_distance_4;
  std_msgs::Float64 target_distance_5;
  std_msgs::Float64 target_distance_6;

  std_msgs::Int32 biggest_cluster_msg;
  std_msgs::Float32MultiArray order_msg;
  std_msgs::Float32MultiArray mean_velocity_msg;
  
  //----------------------------
  // Rate
  ros::Rate loop_rate(30);

  //----------------------------
  // While loop
  while (ros::ok()){

    if(flag_callback_estimation){
      ROS_INFO("1");
      // Classify drone in function of their allocated target
      subswarms_indexes.clear();
      int max_subswarm_size = 0;

      for(int i=0; i<targets_poses.size(); i++){
        int subswarm_size = 0;
        std::vector<int> subswarm_index;
        for(int j=0; j<alloc_array.size(); j++){
          if(getDistance(alloc_array[j], targets_poses[i]) < TARGET_RADIUS){
            subswarm_index.push_back(j);
            subswarm_size += 1;
          }
        }
        if(subswarm_size > max_subswarm_size){
          max_subswarm_size = subswarm_size;
          bigger_cluster = subswarm_size;
        }
        subswarms_indexes.push_back(subswarm_index);
      }

      targets_indexes.clear();
      for(int i=0; i<alloc_array.size(); i++){
        for(int j=0; j<targets_poses.size(); j++){
          if(getDistance(alloc_array[i],targets_poses[j]) < TARGET_RADIUS){
            targets_indexes.push_back(j);
          }
        }
      }

      ROS_INFO("target_pose %d", targets_poses.size());
      ROS_INFO("target_index %d", targets_indexes.size());
      ROS_INFO("2");
      if(targets_indexes.size() > 5){
        // Generate error messages
        ROS_INFO("2_1_1");
        error_drone1 = getDistance(targets_poses[targets_indexes[0]], est_drone1.pose.pose);
        ROS_INFO("2_1_2");
        error_drone2 = getDistance(targets_poses[targets_indexes[1]], est_drone2.pose.pose);
        ROS_INFO("2_1_3");
        error_drone3 = getDistance(targets_poses[targets_indexes[2]], est_drone3.pose.pose);
        ROS_INFO("2_1_4");
        error_drone4 = getDistance(targets_poses[targets_indexes[3]], est_drone4.pose.pose);
        ROS_INFO("2_1_5");
        error_drone5 = getDistance(targets_poses[targets_indexes[4]], est_drone5.pose.pose);
        ROS_INFO("2_1_6");
        error_drone6 = getDistance(targets_poses[targets_indexes[5]], est_drone6.pose.pose);
        ROS_INFO("2_1_7");
        mean_error = (error_drone1 + error_drone2 + error_drone3 + error_drone4 + error_drone5 + error_drone6)/6.0;
        ROS_INFO("2_1_8");
        mean_error_msg.data = mean_error;
        ROS_INFO("2_1");
        error_1_msg.data = targets_indexes[0];
        error_2_msg.data = targets_indexes[1];
        error_3_msg.data = targets_indexes[2];
        error_4_msg.data = targets_indexes[3];
        error_5_msg.data = targets_indexes[4];
        error_6_msg.data = targets_indexes[5];
      
      
        ROS_INFO("2_2");
        pub_mean_error.publish(mean_error_msg);
        pub_error_1.publish(error_1_msg);
        pub_error_2.publish(error_2_msg);
        pub_error_3.publish(error_3_msg);
        pub_error_4.publish(error_4_msg);
        pub_error_5.publish(error_5_msg);
        pub_error_6.publish(error_6_msg);
      }


      if(flag_callback_odom){
        // Compute minimal distance within the swarm
        ROS_INFO("2_3");
        distances_swarm.clear();
        distances_swarm.push_back(getDistance(odom_ptr1.pose.pose,odom_ptr2.pose.pose));
        distances_swarm.push_back(getDistance(odom_ptr1.pose.pose,odom_ptr3.pose.pose));
        distances_swarm.push_back(getDistance(odom_ptr1.pose.pose,odom_ptr4.pose.pose));
        distances_swarm.push_back(getDistance(odom_ptr1.pose.pose,odom_ptr5.pose.pose));
        distances_swarm.push_back(getDistance(odom_ptr1.pose.pose,odom_ptr6.pose.pose));
        distances_swarm.push_back(getDistance(odom_ptr2.pose.pose,odom_ptr3.pose.pose));
        distances_swarm.push_back(getDistance(odom_ptr2.pose.pose,odom_ptr4.pose.pose));
        distances_swarm.push_back(getDistance(odom_ptr2.pose.pose,odom_ptr5.pose.pose));
        distances_swarm.push_back(getDistance(odom_ptr2.pose.pose,odom_ptr6.pose.pose));
        distances_swarm.push_back(getDistance(odom_ptr3.pose.pose,odom_ptr4.pose.pose));
        distances_swarm.push_back(getDistance(odom_ptr3.pose.pose,odom_ptr5.pose.pose));
        distances_swarm.push_back(getDistance(odom_ptr3.pose.pose,odom_ptr6.pose.pose));
        distances_swarm.push_back(getDistance(odom_ptr4.pose.pose,odom_ptr5.pose.pose));
        distances_swarm.push_back(getDistance(odom_ptr4.pose.pose,odom_ptr6.pose.pose));
        distances_swarm.push_back(getDistance(odom_ptr5.pose.pose,odom_ptr6.pose.pose));

        minimal_distance.data = *std::min_element(distances_swarm.begin(), distances_swarm.end());
        ROS_INFO("2_4");
        if(targets_indexes.size() > 5){
          target_distance_1.data = getDistance(odom_ptr1.pose.pose,targets_poses[targets_indexes[0]]);
          target_distance_2.data = getDistance(odom_ptr2.pose.pose,targets_poses[targets_indexes[1]]);
          target_distance_3.data = getDistance(odom_ptr3.pose.pose,targets_poses[targets_indexes[2]]);
          target_distance_4.data = getDistance(odom_ptr4.pose.pose,targets_poses[targets_indexes[3]]);
          target_distance_5.data = getDistance(odom_ptr5.pose.pose,targets_poses[targets_indexes[4]]);
          target_distance_6.data = getDistance(odom_ptr6.pose.pose,targets_poses[targets_indexes[5]]);
        }
        

        ROS_INFO("3");
        mean_velocities.clear();
        orders.clear();
        std::complex<double> arg_sum(0,0);
        for(int i=0; i<subswarms_indexes.size(); i++){
          float subswarm_vel = 0.0;
          arg_sum = std::complex<double>(0.0,0.0);
          for(int j=0; j<subswarms_indexes[i].size(); j++){
            subswarm_vel += std::abs(odom_array[subswarms_indexes[i][j]].twist.twist.linear.x);
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
          subswarm_vel /= subswarms_indexes[i].size();
          mean_velocities.push_back(subswarm_vel);
        }
        ROS_INFO("4");
        //mean_velocity = (odom_array[0].twist.twist.linear.x + odom_array[1].twist.twist.linear.x + odom_array[2].twist.twist.linear.x + odom_array[3].twist.twist.linear.x + odom_array[4].twist.twist.linear.x + odom_array[5].twist.twist.linear.x)/6.0;
        mean_velocity_msg.data = mean_velocities;
        order_msg.data = orders;
        biggest_cluster_msg.data = bigger_cluster;
        

        // Publish messages
        pub_minimal_distance.publish(minimal_distance);
        pub_target_distance_1.publish(target_distance_1);
        pub_target_distance_2.publish(target_distance_2);
        pub_target_distance_3.publish(target_distance_3);
        pub_target_distance_4.publish(target_distance_4);
        pub_target_distance_5.publish(target_distance_5);
        pub_target_distance_6.publish(target_distance_6);

        
        pub_mean_velocity.publish(mean_velocity_msg);
        pub_order.publish(order_msg);
        
        pub_biggest_cluster.publish(biggest_cluster_msg);
        
      }
    }

    ros::spinOnce();

    loop_rate.sleep();
  }
  return 0;
}