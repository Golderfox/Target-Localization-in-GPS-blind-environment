#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Header.h"
#include "std_msgs/Float64.h"

#include <nav_msgs/Odometry.h>
#include "tf/transform_datatypes.h"

#include "stdlib.h"
#include "stdio.h"
#include <string.h>
#include <cmath>

#include <prototype_3/DKF.h>
#include <prototype_3/DKFStamped.h>
#include <prototype_3/DKF_multi.h>
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





#define FALSE 0
#define TRUE 1
#define COM_RANGE 0.5
#define CONTROL_ENABLED 1
#define PUBLISH_INNOVATION 0
#define COEFF_Q 0.000001
#define COEFF_R 0.1

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
int flag_callback_com_out = FALSE;
int flag_callback_swarm = FALSE;
int flag_callback_com_in = FALSE;
int flag_callback_odom = FALSE;

geometry_msgs::PoseStamped sensor_pose;
nav_msgs::Odometry odom_ptr;

std::vector<prototype_3::DKF> communication_data;
std::vector<geometry_msgs::Pose> neighbours;



//----------------------------------------------------------
//                    Defining functions
//----------------------------------------------------------

// --> return the distance between two drones
float getDistance(geometry_msgs::Pose drone_pose1, geometry_msgs::Pose drone_pose2){
  sqrt(std::pow((drone_pose1.position.x)-(drone_pose2.position.x),2.0) + std::pow((drone_pose1.position.y)-(drone_pose2.position.y),2.0) + std::pow((drone_pose1.position.z)-(drone_pose2.position.z),2.0));
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

//----------------------------------------------------------
//                       Flocking functions
//----------------------------------------------------------
double attraction(double p, double p_hat){
  return -(p - p_hat);
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

void odomCallback(const nav_msgs::Odometry& msg)
{
  ROS_INFO("Odometry");
  odom_ptr = msg;
  flag_callback_odom = TRUE;
}

void sensorCallback(const geometry_msgs::PoseStamped& msg)
{
  ROS_INFO("Sensor");
  sensor_pose = msg;
  flag_callback_sensor = TRUE;
}

void commCallback(const prototype_3::DKF_multi& msg)
{
  ROS_INFO("Comm Back");
  communication_data.clear();
  for(int i=0; i<msg.array.size();i++)
  {
    communication_data.push_back(msg.array[i]);
  }
  flag_callback_com_in = TRUE;
}


void neighboursCallback(const geometry_msgs::PoseArray& msg)
{
  ROS_INFO("Neighbours");
  neighbours.clear();
  for(int i=0; i<msg.poses.size();i++)
  {
    neighbours.push_back(msg.poses[i]);
  }
  flag_callback_swarm = TRUE;
}


void Callback(const prototype_3::DKF::ConstPtr drone_1, const prototype_3::DKF::ConstPtr drone_2, const prototype_3::DKF::ConstPtr drone_3)
{
  ROS_INFO("msg received");
}

//----------------------------------------------------------
//                        Main
//----------------------------------------------------------

int main(int argc, char **argv){
  //----------------------------
  // Init
  ros::init(argc, argv, "DKF");

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
  ros::Subscriber sub_odm = n.subscribe("ground_truth/state", 1, odomCallback);
  ros::Subscriber sub_sensor = n.subscribe("target_pose", 1, sensorCallback);
  ros::Subscriber sub_com_input = n.subscribe("input_com", 1, commCallback);
  ros::Subscriber sub_neighbours = n.subscribe("swarm", 1, neighboursCallback);
  /*
  // Neighbours message subscribers
  ros::Subscriber sub_input1 = n.subscribe("/quadrotor1/DKF_out", 1, drone1Callback);
  ros::Subscriber sub_input2 = n.subscribe("/quadrotor2/DKF_out", 1, drone2Callback);
  ros::Subscriber sub_input3 = n.subscribe("/quadrotor3/DKF_out", 1, drone3Callback);
  */

  // Neighbours message publishers
  ros::Publisher pub_drone_pose = n.advertise<geometry_msgs::PoseStamped>("drone_pose",1000);
  ros::Publisher pub_com_output = n.advertise<prototype_3::DKFStamped>("output_com",1000);
  ros::Publisher pub_estimate = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("estimate",1000);
  ros::Publisher pub_innovation = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("innovation",1000);
  ros::Publisher pub_control = n.advertise<geometry_msgs::Twist>("cmd_vel",1000);
  ros::Publisher pub_mag = n.advertise<std_msgs::Float64>("mag",1000);
  ros::Publisher pub_ang = n.advertise<std_msgs::Float64>("ang",1000);
  ros::Publisher pub_X = n.advertise<std_msgs::Float64>("X",1000);
  ros::Publisher pub_Y = n.advertise<std_msgs::Float64>("Y",1000);
  std_msgs::Header header;
  //----------------------------
  // Init matrices for DKF

  // Estimation matrices
  Eigen::MatrixXd x_hat = Eigen::MatrixXd::Zero(3,1);
  Eigen::MatrixXd P = 10000*Eigen::MatrixXd::Identity(3,3);

  Eigen::MatrixXd x_bar = x_hat;
  Eigen::MatrixXd P_bar = P;

  // Noise covariance matrices
  Eigen::MatrixXd Q = COEFF_Q*Eigen::MatrixXd::Identity(3,3);
  Eigen::MatrixXd R = COEFF_R*Eigen::MatrixXd::Identity(3,3);

  // System matrices
  Eigen::MatrixXd A = Eigen::MatrixXd::Identity(3,3);
  Eigen::MatrixXd B = Eigen::MatrixXd::Identity(3,3);;

  // Measurement matrix
  Eigen::MatrixXd y = Eigen::MatrixXd::Zero(3,1);
  Eigen::MatrixXd C = Eigen::MatrixXd::Identity(3,3);

  Eigen::MatrixXd H = C;
  Eigen::MatrixXd U = R;

  Eigen::MatrixXd z = y;

  //----------------------------
  // Set msgs
  prototype_3::DKFStamped com_out_msg;
  prototype_3::DKF DKF_out_msg;

  DKF_out_msg.x_bar_size = 3;
  DKF_out_msg.P_bar_size = 3;
  DKF_out_msg.R_size = 3;

  geometry_msgs::PoseWithCovarianceStamped estimation_msg;
  geometry_msgs::PoseWithCovarianceStamped innovation_msg;

  geometry_msgs::Twist command;




  //----------------------------
  // Rate
  ros::Rate loop_rate(30);

  //----------------------------
  // While loop
  while (ros::ok()){

    // Header
    header.frame_id = "nav";
    header.stamp = ros::Time::now();

    ROS_INFO("time update");
    // Time update step
    x_bar = A*x_hat;
    ROS_INFO("first matrix product");
    P_bar = A*P*A.transpose() + B*Q*B.transpose();

    // Update sensor reading
    if(flag_callback_sensor){
      ROS_INFO("sensor update");
      y << sensor_pose.pose.position.x, sensor_pose.pose.position.y, sensor_pose.pose.position.z;
      H = C;
      U = R;
      z = y;
    }

    ROS_INFO("com step");
    // Communication step
    DKF_out_msg.y = matrixToMsg(y);
    ROS_INFO("first matrix to Msg");
    DKF_out_msg.C = matrixToMsg(C);
    ROS_INFO("2 matrix to Msg");
    DKF_out_msg.R = matrixToMsg(R);
    ROS_INFO("3 matrix to Msg");
    DKF_out_msg.x_bar = matrixToMsg(x_bar);
    ROS_INFO("4 matrix to Msg");
    DKF_out_msg.P_bar = matrixToMsg(P_bar);

    ROS_INFO("msg_com_out");
    com_out_msg.header = header;
    com_out_msg.data = DKF_out_msg;
    pub_com_output.publish(com_out_msg);
    flag_callback_com_out = TRUE;

    ROS_INFO("concat");
    // Concatenation of matrices from other drones in the swarm (we assume that all matrices share the same dimensions for a given type)
    if(flag_callback_com_in){

      // Set H, U and z
      if(flag_callback_sensor){
        H = Eigen::MatrixXd((2*communication_data.size()+1)*C.rows(), C.cols());
        U = Eigen::MatrixXd((2*communication_data.size()+1)*R.rows(), R.cols());
        z = Eigen::MatrixXd((communication_data.size()+1)*y.rows() + communication_data.size()*x_bar.rows(), 1);

        H.block(0,0,C.rows(),C.cols()) = C;
        U.block(0,0,R.rows(),R.cols()) = R;
        H.block(0,0,y.rows(),y.cols()) = y;

        for(int i=0; i<communication_data.size(); i++){
          H.block((i+1)*C.rows(),0,C.rows(),C.cols()) = msgToMatrix(communication_data[i].C, C.rows());
          U.block((i+1)*R.rows(),(i+1)*R.rows(),R.rows(),R.cols()) = msgToMatrix(communication_data[i].R, R.rows());
          z.block((i+1)*y.rows(),0,y.rows(),1) = msgToMatrix(communication_data[i].y, y.rows());
        }
        for(int i=communication_data.size(); i<2*communication_data.size(); i++){
          H.block((i+1)*C.rows(),0,C.rows(),C.cols()) = Eigen::MatrixXd::Identity(C.rows(),C.cols());
          U.block((i+1)*P_bar.rows(),(i+1)*P_bar.rows(),R.rows(),R.cols()) = msgToMatrix(communication_data[i].P_bar, P_bar.rows());
          z.block((i+1)*x_bar.rows(),0,x_bar.rows(),1) = msgToMatrix(communication_data[i].x_bar, x_bar.rows());
        }
      }

      else{
        H = Eigen::MatrixXd((2*communication_data.size())*C.rows(), C.cols());
        U = Eigen::MatrixXd((2*communication_data.size())*R.rows(), R.cols());
        z = Eigen::MatrixXd((communication_data.size())*y.rows() + communication_data.size()*x_bar.rows(), 1);

        for(int i=0; i<communication_data.size(); i++){
          H.block(i*C.rows(),0,C.rows(),C.cols()) = msgToMatrix(communication_data[i].C,C.rows());
          U.block(i*R.rows(),i*R.rows(),R.rows(),R.cols()) = msgToMatrix(communication_data[i].R, R.rows());
          z.block(i*y.rows(),0,y.rows(),1) = msgToMatrix(communication_data[i].y,y.rows());
        }
        for(int i=communication_data.size(); i<2*communication_data.size(); i++){
          H.block(i*C.rows(),0,C.rows(),C.cols()) = Eigen::MatrixXd::Identity(C.rows(),C.cols());
          U.block(i*P_bar.rows(),i*P_bar.rows(),R.rows(),R.cols()) = msgToMatrix(communication_data[i].P_bar, P_bar.rows());
          z.block(i*x_bar.rows(),0,x_bar.rows(),1) = msgToMatrix(communication_data[i].x_bar, x_bar.rows());
        }
      }
    }

    
    // Measurement update step (only if a new measurement is received)
    if(flag_callback_sensor || flag_callback_com_in){
      ROS_INFO("mesurement update");
      x_hat = x_bar + P_bar*H.transpose()*U.inverse()*(z - H*x_bar);
      P = P_bar.inverse() + H.transpose()*U.inverse()*H;
      P = P.inverse();

      
    }
    
  
    ROS_INFO("publication");
    // Publish estimation
    estimation_msg.header = header;
    estimation_msg.pose.pose.position.x = x_hat(0,0);
    estimation_msg.pose.pose.position.y = x_hat(1,0);
    estimation_msg.pose.pose.position.z = x_hat(2,0);

    estimation_msg.pose.pose.orientation.x = 0.0;
    estimation_msg.pose.pose.orientation.y = 0.0;
    estimation_msg.pose.pose.orientation.z = 0.0;
    estimation_msg.pose.pose.orientation.w = 0.0;


    Eigen::MatrixXd E_Cov = Eigen::MatrixXd::Zero(6,6);
    E_Cov.block<3,3>(0,0) = P;
    
    estimation_msg.pose.covariance = matrixToCovMsg(E_Cov);

    pub_estimate.publish(estimation_msg);

    // Publish innovation
    if(flag_callback_com_in || flag_callback_sensor){
      
      innovation_msg.header = header;
      Eigen::MatrixXd innov = z - H*x_bar;
      innovation_msg.pose.pose.position.x = innov(0,0);
      innovation_msg.pose.pose.position.y = innov(1,0);
      innovation_msg.pose.pose.position.z = innov(2,0);

      innovation_msg.pose.pose.orientation.x = 0.0;
      innovation_msg.pose.pose.orientation.y = 0.0;
      innovation_msg.pose.pose.orientation.z = 0.0;
      innovation_msg.pose.pose.orientation.w = 0.0;

      Eigen::MatrixXd I_Cov = Eigen::MatrixXd::Zero(6,6);
      I_Cov.block<3,3>(0,0) = (U + H*P_bar*H.transpose()).block<3,3>(0,0);

      innovation_msg.pose.covariance = matrixToCovMsg(I_Cov);

      if(flag_callback_sensor){
        pub_innovation.publish(innovation_msg);
      }

      // Reset flags
      flag_callback_com_in = FALSE;
      flag_callback_sensor = FALSE;
    }


    // Control
    if(CONTROL_ENABLED){
      if(flag_callback_odom){
        geometry_msgs::PoseStamped drone_pose;
        std_msgs::Float64 mag;
        std_msgs::Float64 ang;
        std_msgs::Float64 X;
        std_msgs::Float64 Y;

        drone_pose.header.stamp = odom_ptr.header.stamp;
        drone_pose.header.frame_id = header.frame_id;
        drone_pose.pose = odom_ptr.pose.pose;
        pub_drone_pose.publish(drone_pose);

        Eigen::Vector2d p(2,1);
        p(0,0) = odom_ptr.pose.pose.position.x;
        p(1,0) = odom_ptr.pose.pose.position.y;

        double x_control; double z_control; double yaw_control;
        Eigen::Vector2d vector_control = Eigen::Vector2d::Zero(2,1);
        //vector_control(0,0) = 0.0;
        //vector_control(1,0) = 0.0;
        
        // Attraction 
        vector_control(0,0) += attraction(odom_ptr.pose.pose.position.x, x_hat(0,0));
        vector_control(1,0) += attraction(odom_ptr.pose.pose.position.y, x_hat(1,0));
        
        
        // Repulsion 
        for(int i=0; i<neighbours.size(); i++){
          Eigen::Vector2d q(2,1);
          q(0,0) = neighbours[i].position.x;
          q(1,0) = neighbours[i].position.y;
          double interspace_distance = (p-q).norm();
          if(odom_ptr.twist.twist.linear.x > 0.05){
            vector_control += 3.0*(p - q)/std::pow(interspace_distance, 3.0);
          }
        }
        
        //----------------------------
        // Create  tf quaternion from pose
        tf::Quaternion q(odom_ptr.pose.pose.orientation.x, odom_ptr.pose.pose.orientation.y, odom_ptr.pose.pose.orientation.z, odom_ptr.pose.pose.orientation.w);
        // Convert quaternion to RPY
        double yaw;
        double pitch;
        double roll;
        tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

        double angle_to_target;
        double magnitude = sqrt(std::pow(vector_control(0,0),2.0) + std::pow(vector_control(1,0),2.0));
        if(vector_control(0,0) == 0.0){
          angle_to_target = 0.0;
        }
        else if(vector_control(0,0) < 0.0){
          angle_to_target = M_PI + atan((vector_control(1,0))/(vector_control(0,0)));
        }
        else{
          angle_to_target = atan((vector_control(1,0))/(vector_control(0,0)));
        }

        // Only for Test 
        mag.data = magnitude;
        ang.data = angle_to_target;
        X.data = vector_control(0,0);
        Y.data = vector_control(1,0);
        pub_mag.publish(mag);
        pub_ang.publish(ang);
        //pub_X.publish(X);
        X.data = 0.0;
        //pub_Y.publish(Y);
        double h_dist = sqrt(std::pow(odom_ptr.pose.pose.position.x - x_hat(0,0),2.0) + std::pow(odom_ptr.pose.pose.position.y - x_hat(1,0),2.0));
        Y.data = h_dist;

        pub_Y.publish(Y);

        if(vector_control(0,0) == 0.0){
          yaw_control = 0.0;
        }
        else{
          yaw_control = COEFF_P_YAW*2*atan(tan((angle_to_target - yaw)/2.0));
        }

        if(h_dist < 2.0){
          x_control = 0.0;
          X.data = 0.0;
        }
        else{
          x_control = COEFF_P_X*atan(magnitude - odom_ptr.twist.twist.linear.x); 
          X.data = 1.0;
        }
        
        z_control = COEFF_P_Z*atan(FLIGHT_HEIGHT - odom_ptr.pose.pose.position.z);

        pub_X.publish(X);
        command.linear.x = x_control;
        command.linear.z = z_control;
        command.angular.z = yaw_control;
        pub_control.publish(command);

        
      }

      
      
    }
    ros::spinOnce();

    loop_rate.sleep();
  }
  return 0;
}