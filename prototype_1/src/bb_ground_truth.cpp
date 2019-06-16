#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Header.h"

#include "sensor_msgs/Image.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <nav_msgs/Odometry.h>

#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/ModelStates.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "stdlib.h"
#include "stdio.h"
#include <string.h>
#include <cmath>
#include "tf/transform_datatypes.h"
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/PointCloud.h>

#define FALSE 0
#define TRUE 1
#define TARGET_KEYWORD "Target"
#define CAMERA_OFFSET_X 0.21
#define CAMERA_OFFSET_Y 0.0
#define CAMERA_OFFSET_Z 0.01

//----------------------------------------------------------
//                       Global variables
//----------------------------------------------------------

//cv_bridge::CvImagePtr cv_ptr;

// Flags used to detect whether suscribers have received any message
int flag_callback = FALSE;
int flag_callback_drone = FALSE;
int flag_callback_mod = FALSE;

// Gazebo Model states msg
gazebo_msgs::ModelStates gt_ptr;
// Odometry msg
nav_msgs::Odometry odom_ptr;

//std_msgs::Header img_header;

// Intrisic matrix of the camera
cv::Mat Intrisic = (cv::Mat_<double>(3,3) << 374.6706070969281, 0.0, 320.5, 0.0, 374.6706070969281, 180.5, 0.0, 0.0, 1.0);
// Vectors
cv::Mat Coord2DH(3,1, CV_64F);
cv::Mat Coord3DH(4,1, CV_64F);
cv::Mat Coord3D_Cam(3,1, CV_64F);

/*
//----------------------------------------------------------
//                    Print functions
//----------------------------------------------------------

// --> print the designated cv::Mat 4x4
void printCVMat4x4(cv::Mat Matrix)
{
  ROS_INFO("Mat = [[%f, %f, %f, %f],\n       [%f, %f, %f, %f],\n       [%f, %f, %f, %f],\n       [%f, %f, %f, %f]]", Matrix.at<double>(0,0),
  Matrix.at<>(0,1), Matrix.at<double>(0,2), Matrix.at<double>(0,3), Matrix.at<double>(1,0), Matrix.at<double>(1,1),
  Matrix.at<double>(1,2), Matrix.at<double>(1,3), Matrix.at<double>(2,0), Matrix.at<double>(2,1), Matrix.at<double>(2,2),
  Matrix.at<double>(2,3), Matrix.at<double>(3,0), Matrix.at<double>(3,1), Matrix.at<double>(3,2), Matrix.at<double>(3,3));
}

// --> print the designated cv::Mat 4x3
void printCVMat3x4(cv::Mat Matrix, std::string Name)
{
  ROS_INFO("\n%s = [[%f, %f, %f, %f],\n       [%f, %f, %f, %f],\n       [%f, %f, %f, %f]]", Name.c_str(),
  Matrix.at<double>(0,0), Matrix.at<double>(0,1), Matrix.at<double>(0,2), Matrix.at<double>(0,3),
  Matrix.at<double>(1,0), Matrix.at<double>(1,1), Matrix.at<double>(1,2), Matrix.at<double>(1,3),
  Matrix.at<double>(2,0), Matrix.at<double>(2,1), Matrix.at<double>(2,2), Matrix.at<double>(2,3));
}
*/
//----------------------------------------------------------
//                    Convertion function
//----------------------------------------------------------

// --> get the extrinsic mat of the front camera from drone's pose
cv::Mat getExtrinsicMatrix(geometry_msgs::Pose msg)
{
  // Initialize Matrix
  cv::Mat Extrinsic(3,4, CV_64F);
  
  //----------------------------
  // Create  tf quaternion from pose
  tf::Quaternion q(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w);

  // Convert quaternion to RPY
  double yaw;
  double pitch;
  double roll;

  tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
  
  //----------------------------
  // Set corrected RPY angles
  yaw += 0.0 - M_PI/2; 
  pitch += 0.0; //-M_PI/2; 
  roll += 0.0 - M_PI/2;

  //----------------------------
  // Set matrices
  cv::Mat Yaw_Mat = (cv::Mat_<double>(3,3) << cos(yaw), -sin(yaw), 0.0, sin(yaw), cos(yaw), 0.0, 0.0, 0.0, 1.0);
  cv::Mat Pitch_Mat = (cv::Mat_<double>(3,3) << cos(pitch), 0.0, sin(pitch), 0.0, 1.0, 0.0, -sin(pitch), 0.0, cos(pitch));
  cv::Mat Roll_Mat = (cv::Mat_<double>(3,3) << 1.0, 0.0, 0.0, 0.0, cos(roll), -sin(roll), 0.0, sin(roll), cos(roll));
  cv::Mat Rot_Mat_T = (Yaw_Mat * Pitch_Mat * Roll_Mat).t();
  cv::Mat Pose_Mat = (cv::Mat_<double>(3,1) << msg.position.x + CAMERA_OFFSET_X, msg.position.y + CAMERA_OFFSET_Y, msg.position.z + CAMERA_OFFSET_Z);
  cv::Mat Pose_corrected_Mat = Rot_Mat_T * Pose_Mat;
  
  // Rotation
  Extrinsic.at<double>(0,0) = Rot_Mat_T.at<double>(0,0); Extrinsic.at<double>(0,1) = Rot_Mat_T.at<double>(0,1); Extrinsic.at<double>(0,2) = Rot_Mat_T.at<double>(0,2);
  Extrinsic.at<double>(1,0) = Rot_Mat_T.at<double>(1,0); Extrinsic.at<double>(1,1) = Rot_Mat_T.at<double>(1,1); Extrinsic.at<double>(1,2) = Rot_Mat_T.at<double>(1,2);
  Extrinsic.at<double>(2,0) = Rot_Mat_T.at<double>(2,0); Extrinsic.at<double>(2,1) = Rot_Mat_T.at<double>(2,1); Extrinsic.at<double>(2,2) = Rot_Mat_T.at<double>(2,2);
  
  // Translation
  Extrinsic.at<double>(0,3) = -1.0 * Pose_corrected_Mat.at<double>(0,0); 
  Extrinsic.at<double>(1,3) = -1.0 * Pose_corrected_Mat.at<double>(1,0); 
  Extrinsic.at<double>(2,3) = -1.0 * Pose_corrected_Mat.at<double>(2,0); 

  return Extrinsic;
}

//----------------------------------------------------------
//                       ImageCallback
//----------------------------------------------------------
/*
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{

  ROS_INFO("IMG");
  img_header = msg->header;
  cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  flag_callback = TRUE;
}
*/
//----------------------------------------------------------
//                       ModelCallback
//----------------------------------------------------------
void modelCallback(const gazebo_msgs::ModelStates& msg)
{

  gt_ptr = msg;
  flag_callback_mod = TRUE;
}

//----------------------------------------------------------
//                       DroneCallback
//----------------------------------------------------------
void droneCallback(const nav_msgs::Odometry& msg)
{
  odom_ptr = msg;
  flag_callback_drone = TRUE;
}

//----------------------------------------------------------
//                        Main
//----------------------------------------------------------

int main(int argc, char **argv)
{
  //----------------------------
  // Init
  ros::init(argc, argv, "bb_ground_truth");

  //----------------------------
  // Node handler with image transport
  ros::NodeHandle n;
  image_transport::ImageTransport it(n);

  //----------------------------
  // Suscribers and publishers
  ros::Subscriber sub_mod = n.subscribe("/gazebo/model_states", 1, modelCallback);
  ros::Subscriber sub_odom = n.subscribe("/quadrotor1/ground_truth/state", 1, droneCallback);
  ros::Publisher pub_rect = n.advertise<geometry_msgs::PolygonStamped>("bounding_boxes_GT",1000);


  //----------------------------
  // Rate
  ros::Rate loop_rate(30);

  //----------------------------
  // While loop
  while (ros::ok())
  {
    // frame link front cam 0.21 0.0 0.01
    char tag[100];
    std_msgs::Header header;
    sensor_msgs::ImagePtr msg_img;
    geometry_msgs::PolygonStamped msg_bounding_rectangles;

    std::vector<std_msgs::String> model_names;
    geometry_msgs::Pose drone_pose;
    std::vector<int> target_indexes;
    int quadrotor_index;

    std::vector<geometry_msgs::Point32> bounding_rectangles;

    // Header
    header.frame_id = "1";
    header.stamp = ros::Time::now();

    if (flag_callback_mod == TRUE && flag_callback_drone == TRUE)
    {
      // Get the indexes of drone and targets
      for (int i=0; i < gt_ptr.name.size(); i++)
      {
        if (((gt_ptr.name[i]).substr(0,6)) == TARGET_KEYWORD)
        {
          target_indexes.push_back(i);
        }
        /*else if((gt_ptr.name[i]) == "quadrotor1")
        {
          quadrotor_index = i;
        }
        */
      }
      
      // Release memory of string vector
      model_names.clear();

      // Set drone matrix transform
      drone_pose = odom_ptr.pose.pose;
      cv::Mat drone_transform = getExtrinsicMatrix(drone_pose);
      cv::Mat drone_projection =  Intrisic * drone_transform;
     
      for(int i=0; i<target_indexes.size(); i++)
      {
        int target_index = target_indexes[i];

        // Get 3D coordinates of the i-th target
        Coord3DH.at<double>(0,0) = gt_ptr.pose[target_index].position.x;
        Coord3DH.at<double>(1,0) = gt_ptr.pose[target_index].position.y;
        Coord3DH.at<double>(2,0) = gt_ptr.pose[target_index].position.z;
        Coord3DH.at<double>(3,0) = 1.0;

        // Create a custom string for the bounding box
        sprintf(tag, "Target %d",i);
        std::string tgt(tag);
        
        // 2D homogenous coordinates of the target
        Coord2DH = drone_projection * Coord3DH;
        // 3D coordinates of the target within the camera's frame
        //Coord3D_Cam = drone_projection * Coord3DH;
        Coord3D_Cam = drone_transform * Coord3DH;

        // Radius of the Target
        float Target_radius = 187.3 * 1/(Coord3D_Cam.at<double>(2,0));

        // Center of 2D point of the target
        cv::Point2f Target_centre((Coord2DH.at<double>(0,0)/Coord2DH.at<double>(2,0)),(Coord2DH.at<double>(1,0)/Coord2DH.at<double>(2,0)));

        // Resulting bounding box
        cv::Rect Bounding_box_rect(Target_centre + cv::Point2f(-Target_radius,-Target_radius),Target_centre + cv::Point2f(Target_radius,Target_radius));
        // Camera image bounding box
        cv::Rect Image_box_rect(cv::Point2f(0.0F, 0.0F), cv::Point2f(640.0F, 360.0F));
        // INtersection between the latters
        cv::Rect Intersection_box_rect = Bounding_box_rect & Image_box_rect;

        // Test whether the target is in front of the camera and inside the picture
        if ((Coord2DH.at<double>(2,0) > 0.0) && (Intersection_box_rect != cv::Rect(cv::Point2f(0.0F, 0.0F), cv::Point2f(0.0F, 0.0F))))//((Bounding_box_rect & cv::Rect(cv::Point2f(0.0F, 0.0F), cv::Point2f(640.0F, 360.0F))) != cv::Rect(cv::Point2f(0.0F, 0.0F), cv::Point2f(0.0F, 0.0F))))
        {
          // Set bounding box left top corner message
          geometry_msgs::Point32 bounding_box_pt;
          bounding_box_pt.x = std::max(0.0F,Target_centre.x - Target_radius);
          bounding_box_pt.y = std::max(0.0F, Target_centre.y - Target_radius);
          bounding_box_pt.z = i;

          // Set bounding box dimensions message
          geometry_msgs::Point32 bounding_box_wh;
          bounding_box_wh.x = Intersection_box_rect.width;
          bounding_box_wh.y = Intersection_box_rect.height;
          bounding_box_wh.z = i;
 
          // Push back of the 2 message points
          bounding_rectangles.push_back(bounding_box_pt);
          bounding_rectangles.push_back(bounding_box_wh);

          /*
          // Draw functions
          cv::rectangle( cv_ptr->image, Target_centre + cv::Point2f(-Target_radius,-Target_radius), Target_centre + cv::Point2f(Target_radius,Target_radius), cv::Scalar( 255, 0, 255), 2, 8, 0 );
          //cv::circle(cv_ptr->image,cv::Point2f((Coord2DH.at<double>(0,0)/Coord2DH.at<double>(2,0)),(Coord2DH.at<double>(1,0)/Coord2DH.at<double>(2,0))),187.3 * 1/(Coord3D_Cam.at<double>(2,0)),cv::Scalar( 255, 0, 255),4);
          //cv::rectangle( cv_ptr->image, Rect_BL, Rect_BL - cv::Point2f(-textSize.width, textSize.height), cv::Scalar(0,0,0), -1, 8, 0 );
          cv::putText(cv_ptr->image, tgt, cv::Point2d((Coord2DH.at<double>(0,0)/Coord2DH.at<double>(2,0)),(Coord2DH.at<double>(1,0)/Coord2DH.at<double>(2,0))), cv::FONT_HERSHEY_PLAIN, 1,  cv::Scalar(255,255,255), 2 , 8 , false);
          */
        }
      }
      // Completing bounding box message
      msg_bounding_rectangles.polygon.points = bounding_rectangles;
      msg_bounding_rectangles.header = odom_ptr.header;

      //msg_img = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cv_ptr->image).toImageMsg();
      ROS_INFO("af_HEAD");
     
    }
    
    // Publishing message
    pub_rect.publish(msg_bounding_rectangles);
    //pub.publish(msg_img);

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;

}
