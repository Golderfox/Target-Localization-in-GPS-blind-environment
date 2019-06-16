#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Header.h"
#include "sensor_msgs/Image.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "stdlib.h"
#include "stdio.h"
#include <string.h>
#include <iostream>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/PolygonStamped.h>

#define FALSE 0
#define TRUE 1

//----------------------------------------------------------
//                       Global variables
//----------------------------------------------------------

// Image pointer
cv_bridge::CvImagePtr cv_ptr;
cv_bridge::CvImage cvi_mat;
std_msgs::Header img_header;
cv::Mat dImg;

// Flag for first message received
int flag_callback = FALSE;


//----------------------------------------------------------
//                       ImageCallback
//----------------------------------------------------------
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{

  img_header = msg->header; 
  cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  flag_callback = TRUE;
}

//----------------------------------------------------------
//                        Main
//----------------------------------------------------------

int main(int argc, char **argv)
{
  char tag[100];
  //----------------------------
  // Init
  ros::init(argc, argv, "tracker");

  //----------------------------
  // Node handler with image transport
  ros::NodeHandle n;
  image_transport::ImageTransport it(n);

  //----------------------------
  // Suscribers and publishers
  //image_transport::Subscriber sub = it.subscribe("/webcam/image_raw", 1, imageCallback);
  image_transport::Subscriber sub = it.subscribe("quadrotor1/ardrone/front/ardrone/front/image_raw", 1, imageCallback);
  image_transport::Publisher pub = it.advertise("out_image_base_topic", 1);
  ros::Publisher pub_rect = n.advertise<geometry_msgs::PolygonStamped>("bounding_boxes_tracker",1000);


  //----------------------------
  // Rate
  ros::Rate loop_rate(10);

  //----------------------------
  // While loop
  while (ros::ok())
  {

    std_msgs::String msg;
    std_msgs::Header header;
    sensor_msgs::ImagePtr msg_img;
    geometry_msgs::PolygonStamped msg_bounding_rectangles;
    std::vector<geometry_msgs::Point32> bounding_rectangles;

    // Header
    header.frame_id = "1";
    header.stamp = ros::Time::now();


    if (flag_callback == TRUE)
    {
      //----------------------------
      // Init the variables

      // Init cv::Mat variables
      cv::Mat frame_HSV;
      cv::Mat frame_threshold;

      // std::vector used for detection of contour
      std::vector<std::vector<cv::Point> > contours;
      std::vector<cv::Vec4i> hierarchy;

      // Set threshold values
      int low_H = 0; int low_S = 10; int low_V = 100;
      int high_H = 40; int high_S = 100; int high_V = 255;

      //----------------------------
      // Thresholding

      // Convert from BGR to HSV colorspace
      cv::cvtColor(cv_ptr->image, frame_HSV, cv::COLOR_BGR2HSV);

      cv::inRange(frame_HSV, cv::Scalar(0, 100, 30), cv::Scalar(30, 255, 255), frame_threshold);

      //----------------------------
      // Find contours operations

      // Morphological opening 
      cv::erode(frame_threshold, frame_threshold, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );
      cv::dilate( frame_threshold, frame_threshold, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) ); 

      // Morphological closing (removes small holes from the foreground)
      cv::dilate( frame_threshold, frame_threshold, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) ); 
      cv::erode(frame_threshold, frame_threshold, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );
  
      // Find contours
      cv::findContours(frame_threshold, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );

      // Approximate contours to polygons + get bounding rects and circles
      std::vector<std::vector<cv::Point> > contours_poly( contours.size() );
      std::vector<cv::Rect> boundRect( contours.size() );
      std::vector<cv::Point2f>center( contours.size() );


      for( int i = 0; i < contours.size(); i++ )
      { 
        boundRect[i] = cv::boundingRect(contours[i]);
      }

      //----------------------------
      // Set bounding rectangles message
      for( int i = 0; i< contours.size(); i++ )
      {
       // Set bounding box left top corner point
        geometry_msgs::Point32 bounding_box_pt;
        geometry_msgs::Point32 bounding_box_wh;
        bounding_box_pt.x = boundRect[i].x;
        bounding_box_pt.y = boundRect[i].y;
        bounding_box_pt.z = i; 

        // Set bounding box dimensions
        bounding_box_wh.x = boundRect[i].width;
        bounding_box_wh.y = boundRect[i].height;
        bounding_box_wh.z = i; 

        // Push back
        bounding_rectangles.push_back(bounding_box_pt);
        bounding_rectangles.push_back(bounding_box_wh);

        // Labelling the target
        sprintf(tag, "Target %d",i+1);
        std::string tgt(tag);
        memset(tag, 0, 100 * sizeof(char)); 

        /*
        // Draw functions
        cv::Scalar color = cv::Scalar( 0, 255, 0);
        cv::Point2f Rect_BL = cv::Point2f(boundRect[i].x,boundRect[i].y + boundRect[i].height);
        cv::Size textSize = cv::getTextSize(tgt, cv::FONT_HERSHEY_PLAIN, 1, 2, 0 );
        cv::rectangle( cv_ptr->image, boundRect[i].tl(), boundRect[i].br(), color, 2, 8, 0 );
        cv::rectangle( cv_ptr->image, Rect_BL, Rect_BL - cv::Point2f(-textSize.width, textSize.height), cv::Scalar(0,0,0), -1, 8, 0 );
        cv::putText(cv_ptr->image, tgt, Rect_BL, cv::FONT_HERSHEY_PLAIN, 1,  cv::Scalar(255,255,255), 2 , 8 , false);
        */
     }

      cv::cvtColor(frame_threshold, cv_ptr->image, cv::COLOR_GRAY2BGR);
      //----------------------------
      // Set bounding boxes message
      msg_img = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cv_ptr->image).toImageMsg();
      msg_bounding_rectangles.polygon.points = bounding_rectangles;
      msg_bounding_rectangles.header = img_header;
    }
    // Publishing message
    pub_rect.publish(msg_bounding_rectangles);
    pub.publish(msg_img);
    ros::spinOnce();

    loop_rate.sleep();
  }
  return 0;

}
