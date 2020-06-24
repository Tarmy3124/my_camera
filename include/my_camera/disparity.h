#ifndef DISPARITY_H
#define DISPARITY_H
#include <boost/shared_ptr.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/video.hpp>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

  // Subscribers and publishers.
  //message_filters::Subscriber<
    //sensor_msgs::Image> cam0_img_sub;
  //message_filters::Subscriber<
   // sensor_msgs::Image> cam1_img_sub;
  //message_filters::TimeSynchronizer<
    //sensor_msgs::Image, sensor_msgs::Image> stereo_sub;

namespace cv {

class StereoSGBM;
class StereoBM;
}  // namespace cv

cv::Ptr<cv::StereoSGBM> sgbm_matcher;
#endif 
