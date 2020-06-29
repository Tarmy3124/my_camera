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

/*void disparity_processor()
{
    cv::cvtColor(img_rgb1, img_rgb1, CV_BGR2GRAY);
    cv::cvtColor(img_rgb2, img_rgb2, CV_BGR2GRAY);
    //设置参数
    int numberOfDisparities = 48, SADWindowSize = 11;
    int uniquenessRatio = 15, speckleWindowSize = 50, speckleRange = 32;
    cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create (0,
numberOfDisparities, SADWindowSize);
    int cn = img_rgb1.channels();
    sgbm->setP1(8 * cn * SADWindowSize * SADWindowSize);
    sgbm->setP2(32 * cn * SADWindowSize * SADWindowSize);
    sgbm->setPreFilterCap(63);
    sgbm->setUniquenessRatio(uniquenessRatio);
    sgbm->setSpeckleWindowSize(speckleWindowSize);
    sgbm->setSpeckleRange(speckleRange);
    sgbm->setDisp12MaxDiff(1);
//计算并显示视差
    Mat disp;
    sgbm->compute(img_rgb1, img_rgb2, disp);
    disp.convertTo(disp, CV_8U, 255 / (numberOfDisparities*16.));   //将16位符号整形的视差矩阵转换为8位无符号整形矩阵
    cv::imshow("disparity", disp);
    //cv::waitKey();
}*/

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
   // cv::cvtColor(img_rgb1, img_rgb1, cv::COLOR_RGB2GRAY);
   // cv::cvtColor(img_rgb2, img_rgb2, cv::COLOR_RGB2GRAY);
//ROS_INFO("It's all right at the moment");
    //设置参数
    int numberOfDisparities = 48, SADWindowSize = 11;
    int uniquenessRatio = 15, speckleWindowSize = 50, speckleRange = 32;
    cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create (0,
numberOfDisparities, SADWindowSize);
    int cn = img_rgb1.channels();
    sgbm->setP1(8 * cn * SADWindowSize * SADWindowSize);
    sgbm->setP2(32 * cn * SADWindowSize * SADWindowSize);
    sgbm->setPreFilterCap(63);
    sgbm->setUniquenessRatio(uniquenessRatio);
    sgbm->setSpeckleWindowSize(speckleWindowSize);
    sgbm->setSpeckleRange(speckleRange);
    sgbm->setDisp12MaxDiff(1);
//计算并显示视差
    cv::Mat disp;
    sgbm->compute(img_rgb1, img_rgb2, disp);
    disp.convertTo(disp, CV_8U, 255 / (numberOfDisparities*16.));   //将16位符号整形的视差矩阵转换为8位无符号整形矩阵
    cv::imshow("disparity", disp);
  cv::waitKey(5);
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
