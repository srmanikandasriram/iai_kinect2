#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <thread>
#include <mutex>
#include <chrono>
#include <sys/stat.h>

#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/SetCameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <compressed_depth_image_transport/compression_common.h>

#include <kinect2_bridge/kinect2_definitions.h>
#include <kinect2_registration/kinect2_registration.h>

#include <tf/transform_broadcaster.h>

cv::Size sizeColor, sizeIr, sizeLowRes;
cv::Mat color, ir, depth;
cv::Mat cameraMatrixColor, distortionColor, cameraMatrixLowRes, cameraMatrixIr, distortionIr;
cv::Mat rotation, translation;
cv::Mat map1Color, map2Color, map1Ir, map2Ir, map1LowRes, map2LowRes;

DepthRegistration *depthRegLowRes, *depthRegHighRes;
std::string method;

image_transport::Publisher depth_rect_pub, depth_rect_high_res_pub;

// std::thread tfPublisher;

// void publishStaticTF()
// {
//   tf::TransformBroadcaster broadcaster;
//   tf::StampedTransform stColorOpt, stIrOpt;
//   ros::Time now = ros::Time::now();

//   tf::Matrix3x3 rot(rotation.at<double>(0, 0), rotation.at<double>(0, 1), rotation.at<double>(0, 2),
//                     rotation.at<double>(1, 0), rotation.at<double>(1, 1), rotation.at<double>(1, 2),
//                     rotation.at<double>(2, 0), rotation.at<double>(2, 1), rotation.at<double>(2, 2));

//   tf::Quaternion qZero;
//   qZero.setRPY(0, 0, 0);
//   tf::Vector3 trans(translation.at<double>(0), translation.at<double>(1), translation.at<double>(2));
//   tf::Vector3 vZero(0, 0, 0);
//   tf::Transform tIr(rot, trans), tZero(qZero, vZero);
//   std::string baseNameTF = K2_DEFAULT_NS;
//   stColorOpt = tf::StampedTransform(tZero, now, baseNameTF + K2_TF_LINK, baseNameTF + K2_TF_RGB_OPT_FRAME);
//   stIrOpt = tf::StampedTransform(tIr, now, baseNameTF + K2_TF_RGB_OPT_FRAME, baseNameTF + K2_TF_IR_OPT_FRAME);

//   while(ros::ok())
//   {
//     now = ros::Time::now();
//     stColorOpt.stamp_ = now;
//     stIrOpt.stamp_ = now;

//     broadcaster.sendTransform(stColorOpt);
//     broadcaster.sendTransform(stIrOpt);
//     std::this_thread::sleep_for(std::chrono::milliseconds(10));
//   }
// }

void depthCallback(const sensor_msgs::ImageConstPtr& depth)
{
  try
  {
    cv::Mat depthShifted (cv_bridge::toCvShare(depth, depth->encoding.c_str())->image);
    cv::Mat depth_rect;
    // ROS_INFO("encoding is %s", depth->encoding.c_str());
    depthRegLowRes->registerDepth(depthShifted,depth_rect);
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(depth->header, depth->encoding.c_str(), depth_rect).toImageMsg();
    depth_rect_pub.publish(msg);
    // cv::Mat depth_rect_high_res;
    // depthRegHighRes->registerDepth(depthShifted,depth_rect_high_res);
    // sensor_msgs::ImagePtr msg_high_res = cv_bridge::CvImage(depth->header, depth->encoding.c_str(), depth_rect_high_res).toImageMsg();
    // depth_rect_high_res_pub.publish(msg_high_res);

  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to '16UC1'.", depth->encoding.c_str());
  }

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "depth_rect");
  ros::NodeHandle nh;
  // cv::namedWindow("view");
  // cv::startWindowThread();
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber depth_sub = it.subscribe("/kinect2/qhd/image_depth", 1, depthCallback);

  if(!ros::ok())
  {
    std::cerr << "ros::ok failed!" << std::endl;
    return -1;
  }
  
  sizeColor  = cv::Size(1920, 1080);
  sizeIr     = cv::Size(512, 424);
  sizeLowRes = cv::Size(sizeColor.width/2, sizeColor.height/2);

  cameraMatrixColor = cv::Mat::eye(3, 3, CV_64F);
  distortionColor = cv::Mat::zeros(1, 5, CV_64F);

  cameraMatrixColor.at<double>(0, 0) = 1081.37;
  cameraMatrixColor.at<double>(1, 1) = 1081.37;
  cameraMatrixColor.at<double>(0, 2) = 959.5;
  cameraMatrixColor.at<double>(1, 2) = 539.5;
  cameraMatrixColor.at<double>(2, 2) = 1;

  cameraMatrixIr = cv::Mat::eye(3, 3, CV_64F);
  distortionIr = cv::Mat::zeros(1, 5, CV_64F);

  cameraMatrixIr.at<double>(0, 0) = 364.68;
  cameraMatrixIr.at<double>(1, 1) = 364.68;
  cameraMatrixIr.at<double>(0, 2) = 256.453;
  cameraMatrixIr.at<double>(1, 2) = 209.722;
  cameraMatrixIr.at<double>(2, 2) = 1;

  distortionIr.at<double>(0, 0) = 0.0996511;
  distortionIr.at<double>(0, 1) = -0.269812;
  distortionIr.at<double>(0, 2) = 0;
  distortionIr.at<double>(0, 3) = 0;
  distortionIr.at<double>(0, 4) = 0.0853146;

  rotation = cv::Mat::eye(3, 3, CV_64F);
  translation = cv::Mat::zeros(3, 1, CV_64F);
  translation.at<double>(0) = -0.0520;

  cameraMatrixLowRes = cameraMatrixColor.clone();
  cameraMatrixLowRes.at<double>(0, 0) /= 2;
  cameraMatrixLowRes.at<double>(1, 1) /= 2;
  cameraMatrixLowRes.at<double>(0, 2) /= 2;
  cameraMatrixLowRes.at<double>(1, 2) /= 2;

  const int mapType = CV_16SC2;
  cv::initUndistortRectifyMap(cameraMatrixColor, distortionColor, cv::Mat(), cameraMatrixColor, sizeColor, mapType, map1Color, map2Color);
  cv::initUndistortRectifyMap(cameraMatrixIr, distortionIr, cv::Mat(), cameraMatrixIr, sizeIr, mapType, map1Ir, map2Ir);
  cv::initUndistortRectifyMap(cameraMatrixColor, distortionColor, cv::Mat(), cameraMatrixLowRes, sizeLowRes, mapType, map1LowRes, map2LowRes);

  method = "opencl";
  DepthRegistration::Method reg;

  if(method == "default")
  {
    reg = DepthRegistration::DEFAULT;
  }
  else if(method == "cpu")
  {
#ifdef DEPTH_REG_CPU
    reg = DepthRegistration::CPU;
#else
    std::cerr << "CPU registration is not available!" << std::endl;
    return -1;
#endif
  }
  else if(method == "opencl")
  {
#ifdef DEPTH_REG_OPENCL
    reg = DepthRegistration::OPENCL;
#else
    std::cerr << "OpenCL registration is not available!" << std::endl;
    return -1;
#endif
  }
  else
  {
    std::cerr << "Unknown registration method: " << method << std::endl;
    return false;
  }

  depthRegLowRes = DepthRegistration::New(reg);
  depthRegHighRes = DepthRegistration::New(reg);

  bool ret = true;
  ret = ret && depthRegLowRes->init(cameraMatrixLowRes, sizeLowRes, cameraMatrixIr, sizeIr, distortionIr, rotation, translation, 0.5f, 12.0f);
  ret = ret && depthRegHighRes->init(cameraMatrixColor, sizeColor, cameraMatrixIr, sizeIr, distortionIr, rotation, translation, 0.5f, 12.0f);

  if(!ret)
    return -1;

  depth_rect_pub = it.advertise("/kinect2/qhd/image_depth_rect", 2);
  depth_rect_high_res_pub = it.advertise("/kinect2/hd/image_depth_rect", 2);

  // tfPublisher = std::thread(&publishStaticTF);

  ros::spin();
  // tfPublisher.join();
  ros::shutdown();
  // cv::destroyWindow("view");
  return 0;
}
