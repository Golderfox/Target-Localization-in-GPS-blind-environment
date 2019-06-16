#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Header.h"
#include "sensor_msgs/Image.h"
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "stdlib.h"
#include "stdio.h"
#include <string.h>
#include <iostream>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/Point32.h>

#define FALSE 0
#define TRUE 1

//----------------------------------------------------------
//                       Global variables
//----------------------------------------------------------

cv_bridge::CvImagePtr cv_ptr;
cv_bridge::CvImage cvi_mat;
cv::Mat dImg;
std_msgs::Header bb_header;

// Flag for first message received
int flag_callback_Tracker = FALSE;
int flag_callback_GT = FALSE;
int flag_callback = FALSE;

// Numbers of false positve, true positive and false negative
int Nfp = 0; int Ntp = 0; int Nfn = 0;


std::vector<cv::Rect> Tracker_bounding_boxes;
std::vector<cv::Rect> GT_bounding_boxes;

/*
//----------------------------------------------------------
//                       BoundingBoxesCallbacks
//----------------------------------------------------------
void boundingBoxGTCallback(const geometry_msgs::PolygonStamped& msg)
{
  GT_bounding_boxes.clear();
  bb_header = msg.header;
  flag_callback_GT = TRUE;
  ROS_INFO("GT");
  for(int i=0; i<msg.polygon.points.size(); i++)
  {
    GT_bounding_boxes.push_back(cv::Rect(msg.polygon.points[i].x, msg.polygon.points[i].y, msg.polygon.points[i].z, msg.polygon.points[i].z));
  }

  
}

void boundingBoxTrackerCallback(const geometry_msgs::PolygonStamped& msg)
{
  Tracker_bounding_boxes.clear();
  flag_callback_Tracker = TRUE;
  ROS_INFO("Tracker");
  for(int i=0; i<msg.polygon.points.size(); i++)
  {
    Tracker_bounding_boxes.push_back(cv::Rect(msg.polygon.points[i].x, msg.polygon.points[i].y, msg.polygon.points[i].z, msg.polygon.points[i].z));
  }
}


//----------------------------------------------------------
//                       ImageCallback
//----------------------------------------------------------
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{

  ROS_INFO("IMG");
  cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  flag_callback = TRUE;
  ROS_INFO("af IMG");
}
*/

//----------------------------------------------------------
//                       Messages Callback
//----------------------------------------------------------

void Callback(const sensor_msgs::Image::ConstPtr msgImg, const geometry_msgs::PolygonStamped::ConstPtr msgBB, const geometry_msgs::PolygonStamped::ConstPtr msgTracker)
{
  // Set flag
  flag_callback = TRUE;

  // Clear bounding boxes vectors
  Tracker_bounding_boxes.clear();
  GT_bounding_boxes.clear();

  // Convert point messages into cv::Rect and push them back in the previous vectors
  cv_ptr = cv_bridge::toCvCopy(msgImg, sensor_msgs::image_encodings::BGR8);
  for(int i=0; i<msgTracker->polygon.points.size()/2; i++)
  {
    Tracker_bounding_boxes.push_back(cv::Rect(msgTracker->polygon.points[2*i].x, msgTracker->polygon.points[2*i].y, msgTracker->polygon.points[2*i+1].x, msgTracker->polygon.points[2*i+1].y));
  }
  for(int i=0; i<msgBB->polygon.points.size()/2; i++)
  {
    GT_bounding_boxes.push_back(cv::Rect(msgBB->polygon.points[2*i].x, msgBB->polygon.points[2*i].y, msgBB->polygon.points[2*i+1].x, msgBB->polygon.points[2*i+1].y));
  }
}

//----------------------------------------------------------
//                        Main
//----------------------------------------------------------

int main(int argc, char **argv)
{
  char tag[100];
  //----------------------------
  // Init
  ros::init(argc, argv, "visualization");

  //----------------------------
  // Node handler with image transport
  ros::NodeHandle n;
  image_transport::ImageTransport it(n);

  //----------------------------
  // Suscribers and publishers
  image_transport::Publisher pub = it.advertise("out_image", 1);
  message_filters::Subscriber<geometry_msgs::PolygonStamped> sub_bb_GT(n, "/bounding_boxes_GT", 1);
  message_filters::Subscriber<geometry_msgs::PolygonStamped> sub_bb_tracker(n, "/bounding_boxes_tracker", 1);
  image_transport::SubscriberFilter sub_img(it, "quadrotor1/ardrone/front/ardrone/front/image_raw", 1);

  // Message synchronizer
  typedef message_filters::sync_policies::ApproximateTime< sensor_msgs::Image, geometry_msgs::PolygonStamped, geometry_msgs::PolygonStamped> MySyncPolicy;
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10),  sub_img, sub_bb_GT, sub_bb_tracker);
  sync.registerCallback(boost::bind(&Callback, _1, _2, _3));

  //----------------------------
  // Rate
  ros::Rate loop_rate(10);

  //----------------------------
  // While loop
  while (ros::ok())
  {
    //----------------------------
    // Init variables
    char tag[100];
    std_msgs::Header header;
    sensor_msgs::ImagePtr msg_img;

    // Header
    header.frame_id = "1";
    header.stamp = ros::Time::now();

    //----------------------------
    // Draw bounding boxes
    if(flag_callback == TRUE)
    {
      for(int i=0; i<GT_bounding_boxes.size(); i++)
      {
        // Draw ground truth bounding box
        cv::rectangle( cv_ptr->image, GT_bounding_boxes[i], cv::Scalar( 255, 0, 0), 2, 8, 0);
          
      }
      for(int i=0; i<Tracker_bounding_boxes.size(); i++)
      {
        // Draw tracker bounding box
        sprintf(tag, "Target %d",i);
        std::string tgt(tag);
        memset(tag, 0, 100 * sizeof(char)); 
        cv::Point2f Rect_BL(Tracker_bounding_boxes[i].tl().x, Tracker_bounding_boxes[i].tl().y + Tracker_bounding_boxes[i].height);
        cv::Size textSize = cv::getTextSize(tgt, cv::FONT_HERSHEY_PLAIN, 1, 2, 0 );
        cv::rectangle(cv_ptr->image, Tracker_bounding_boxes[i], cv::Scalar( 0, 255, 0), 2, 8, 0);
        cv::rectangle( cv_ptr->image, Rect_BL, Rect_BL - cv::Point2f(-textSize.width, textSize.height), cv::Scalar(0,0,0), -1, 8, 0 );
        cv::putText(cv_ptr->image, tgt, Rect_BL, cv::FONT_HERSHEY_PLAIN, 1,  cv::Scalar(255,255,255), 2 , 8 , false);
      }

      //----------------------------
      // Create image message
      msg_img = cv_bridge::CvImage(bb_header, "bgr8", cv_ptr->image).toImageMsg();
    
      // Publish message
      pub.publish(msg_img);

      flag_callback = FALSE;
      
    }
      
    
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;

}
