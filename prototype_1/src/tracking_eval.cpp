#include "ros/ros.h"
#include "std_msgs/String.h"
#include <std_msgs/Float32.h>
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
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/Point32.h>
#include <numeric>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#define FALSE 0
#define TRUE 1

// Macros of tracking functions
#define Precision(x,y,z) (((float)x)/(((float)x) + ((float)y)))
#define Recall(x,y,z) (((float)x)/(((float)x) + ((float)z)))
#define F_score(x,y) ((x)*(y))/((x)+(y))

//----------------------------------------------------------
//                       Global variables
//----------------------------------------------------------

cv_bridge::CvImagePtr cv_ptr;
cv_bridge::CvImage cvi_mat;
cv::Mat dImg;

// Flags for first message received
int flag_callback_Tracker = FALSE;
int flag_callback_GT = FALSE;
int flag_callback = FALSE;

// Numbers of false positve, true positive and false negative
int Nfp = 0; int Ntp = 0; int Nfn = 0; int Ms = 0;

std::vector<cv::Rect> Tracker_bounding_boxes;
std::vector<cv::Rect> GT_bounding_boxes;

// std::vector of distances for deviation evaluation
std::vector<float> Distances;


//----------------------------------------------------------
//                       Evaluation functions
//----------------------------------------------------------

// Pascal criterion

int Pascal_criterion(cv::Rect rect1, cv::Rect rect2)
{
  if((rect1.tl().x > rect2.br().x) || (rect2.tl().x > rect1.br().x))
  {
    return 0;
  }
  else
  {
    int inter; int uni;
    if(rect1.tl().x < rect2.tl().x)
    {
      if(rect1.tl().y < rect1.tl().y)
      {
        inter = (rect1.br() - rect2.tl()).x * (rect1.br() - rect2.tl()).y;
      }
      else
      {
        inter = (rect1.br() - rect2.tl()).x * (rect2.br() - rect1.tl()).y;
      }
    }
    else
    {
      if(rect1.tl().y > rect1.tl().y)
      {
        inter = (rect2.br() - rect1.tl()).x * (rect2.br() - rect1.tl()).y;
      }
      else
      {
        inter = (rect2.br() - rect1.tl()).x * (rect1.br() - rect2.tl()).y;
      }
    }
    uni = (rect1.width * rect1.height) + (rect2.width * rect2.height) - inter;

    if (inter/uni >= 0.5)
    {
      return 1;
    }
    else
    {
      return 0;
    }  
  }
}


/*
float precision(int ntp,int nfp,int nfn)
{
  return (float)ntp/((float)ntp+(float)nfp);
}

float recall(int ntp,int nfp,int nfn)
{
  return (float)ntp/((float)ntp+(float)nfn);
}

float F_score(float precision, float recall)
{
  return 2* precision*recall/(precision + recall);
}
*/
// Function which compute the distance between two cv::Rect
float distance(cv::Rect rectangle1, cv::Rect rectangle2)
{
  return sqrt(std::pow((rectangle1.tl().x + rectangle1.width/2)-(rectangle2.tl().x + rectangle2.width/2),2.0) + std::pow((rectangle1.tl().y + rectangle1.height/2)-(rectangle2.tl().y + rectangle2.height/2),2.0));
}

// Deviation
float deviation(int Ms, std::vector<float> distances)
{
  return 1 - std::accumulate(distances.begin(), distances.end(), 0.0)/Ms;
}

//----------------------------------------------------------
//                     BoundingBoxesCallbacks
//----------------------------------------------------------
/*
void boundingBoxGTCallback(const geometry_msgs::PolygonStamped& msg)
{
  GT_bounding_boxes.clear();
  flag_callback_GT = TRUE;
  for(int i=0; i<msg.polygon.points.size(); i++)
  {
    GT_bounding_boxes.push_back(cv::Rect(msg.polygon.points[i].x, msg.polygon.points[i].y, msg.polygon.points[i].z, msg.polygon.points[i].z));
  }
}

void boundingBoxTrackerCallback(const geometry_msgs::PolygonStamped& msg)
{
  Tracker_bounding_boxes.clear();
  flag_callback_Tracker = TRUE;
  for(int i=0; i<msg.polygon.points.size(); i++)
  {
    Tracker_bounding_boxes.push_back(cv::Rect(msg.polygon.points[i].x, msg.polygon.points[i].y, msg.polygon.points[i].z, msg.polygon.points[i].z));
  }
}
*/
void boundingBoxesCallback(const geometry_msgs::PolygonStamped::ConstPtr msgBB, const geometry_msgs::PolygonStamped::ConstPtr msgTracker)
{
  // Set flag
  flag_callback = TRUE;

  // Clear bounding boxes vectors
  Tracker_bounding_boxes.clear();
  GT_bounding_boxes.clear();

  // Convert point messages into cv::Rect and push them back in the previous vectors
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
  ros::init(argc, argv, "tracking_evaluation");

  //----------------------------
  // Node handler 
  ros::NodeHandle n;

  //----------------------------
  // Suscribers and publishers
  message_filters::Subscriber<geometry_msgs::PolygonStamped> sub_bb_GT(n, "/bounding_boxes_GT", 1);
  message_filters::Subscriber<geometry_msgs::PolygonStamped> sub_bb_tracker(n, "/bounding_boxes_tracker", 1);
  typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PolygonStamped, geometry_msgs::PolygonStamped> MySyncPolicy;

  // Message synchronizer
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sub_bb_GT, sub_bb_tracker);
  sync.registerCallback(boost::bind(&boundingBoxesCallback, _1, _2));
  ros::Publisher pub_F_score = n.advertise<std_msgs::Float32>("F_score",1000);
  ros::Publisher pub_deviation = n.advertise<std_msgs::Float32>("deviation",1000);


  //----------------------------
  // Rate
  ros::Rate loop_rate(10);

  //----------------------------
  // While loop
  while (ros::ok())
  {
    //ROS_INFO("------------------------------------------- \n Ntp: %d | Nfp: %d | Nfn: %d", Ntp, Nfp, Nfn);
    // Case where 
    if(Ntp > 0 || Nfp > 0)
    {
      float rec = Recall(Ntp, Nfp, Nfn);
      float pre = Precision(Ntp, Nfp, Nfn);
      ROS_INFO("pr: %f | rec: %f \n ----------------------------------------------", pre, rec);
    }
    
    std_msgs::Float32 Accuracy;
    std_msgs::Float32 Deviation;
    
    
    if (flag_callback == TRUE) //(flag_callback_GT == TRUE && flag_callback_Tracker == TRUE)
    {
      
      if(GT_bounding_boxes.size() > 0 && Tracker_bounding_boxes.size() > 0)
      {
        if(Pascal_criterion(GT_bounding_boxes[0], Tracker_bounding_boxes[0]))
        {
          Ntp += 1;
          Ms += 1;
        }
        else
        {
          Nfp += 1;
          Nfn += 1;
        }
        Distances.push_back(distance(GT_bounding_boxes[0], Tracker_bounding_boxes[0]));
      }
      else if(GT_bounding_boxes.size() > 0 )
      {
        Nfn += 1;
      }
      else if(Tracker_bounding_boxes.size() > 0)
      {
        Nfp += 1;
      }
      Accuracy.data = F_score(Precision(Ntp, Nfp, Nfn), Recall(Ntp, Nfp, Nfn));
      Deviation.data = deviation(Ms, Distances);
    }
    
    pub_F_score.publish(Accuracy);
    pub_deviation.publish(Deviation);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;

}
