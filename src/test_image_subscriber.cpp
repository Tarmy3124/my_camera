//test_image_subscriber.cpp
#include <ros/ros.h>  
#include <image_transport/image_transport.h>  
#include <opencv2/highgui/highgui.hpp>  
#include <cv_bridge/cv_bridge.h>  



#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/persistence.hpp>

#include "my_camera/disparity.h"
//using namespace cv;
using namespace sensor_msgs;
using namespace message_filters;
void imageCallback(const sensor_msgs::ImageConstPtr& msg1,
                   const sensor_msgs::ImageConstPtr& msg2)

{  

  cv_bridge::CvImagePtr cv_ptr1,cv_ptr2;  
  try  
  {  
     cv_ptr1 = cv_bridge::toCvCopy(msg1, "mono8");
     cv_ptr2 = cv_bridge::toCvCopy(msg2, "mono8");  
  }  
  catch (cv_bridge::Exception& e)  
  {  
     ROS_ERROR("cv_bridge exception: %s", e.what());  
     return;  
  }  

  cv::Mat img_rgb1, img_rgb2;  
  img_rgb1 = cv_ptr1->image; 
  img_rgb2 = cv_ptr2->image; 

  cv::imshow("cam0", img_rgb1); 
  cv::imshow("cam1", img_rgb2); 
  // fail if don't have waitKey(3).
  cv::waitKey(3);
}  

int main(int argc, char **argv)  
{  
  ros::init(argc, argv, "image_subscriber");  
  ros::NodeHandle nh;  
  cv::namedWindow("cam0", CV_WINDOW_NORMAL);
  cv::namedWindow("cam1", CV_WINDOW_NORMAL);   
  cv::startWindowThread();  
///
//  image_transport::ImageTransport it(nh);  
//  image_transport::Subscriber sub = it.subscribe("/cam1/image_raw", 1, /////imageCallback);

//  cam0_img_sub.subscribe(nh, "/cam0/image_raw", 10);
//  cam1_img_sub.subscribe(nh, "/cam1/image_raw", 10);
   message_filters::Subscriber<Image> cam0_img_sub(nh, "/cam0/image_raw", 1);
   message_filters::Subscriber<Image> cam1_img_sub(nh, "/cam1/image_raw", 1);
   
  TimeSynchronizer<Image, Image> sync(cam0_img_sub, cam1_img_sub, 10);
  sync.registerCallback(boost::bind(&imageCallback, _1, _2));
  //sync.registerCallback(imageCallback);
  ros::spin();  
  //cv::destroyWindow("view1");  
  return 0;
} 
