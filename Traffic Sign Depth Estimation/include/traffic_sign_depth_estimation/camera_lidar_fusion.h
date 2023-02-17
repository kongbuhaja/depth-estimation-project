/**
 * @file camera_lidar_fusion.h
 * @author Seungho Hyeong (slkumquat@gmail.com)
 * @brief Camera Lidar Fusion Class header file
 * @version 1.0
 * @date 2023-02-15
 */
#ifndef CAMER_LIDAR_FUSION_H_
#define CAMER_LIDAR_FUSION_H_

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>
#include <yaml-cpp/yaml.h>
#include <vector>
#include <string>
#include <math.h>


#include "traffic_sign_depth_estimation/camera_lidar_fusion.h"
#include "traffic_sign_depth_estimation/camera_depth_estimation.h"
#include "traffic_sign_depth_estimation/lidar_depth_estimation.h"

#include "opencv2/opencv.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/calib3d/calib3d_c.h"

#include "yolov3_trt_ros/BoundingBox.h"
#include "yolov3_trt_ros/BoundingBoxes.h"


struct DetectionObject
{
  int id = -1;
  float probability;
  int xmin;
  int ymin;
  int xmax;
  int ymax;
  int area;
};

class CameraLidarFusion {
public:
  // Construct a new Camera Lidar Fusion System object
    CameraLidarFusion();
  // Destroy the Camera Lidar Fusion object
  virtual ~CameraLidarFusion();
  // Running Camera Lidar Fusion Algorithm
  void run();
  // Set parameters from config file
  void setParams(const YAML::Node &config);

private:
  // Subcribe Image Topic
  void imageCallback(const sensor_msgs::Image &msg);

  // Subscribe Yolo Topic
  void yoloCallback(const yolov3_trt_ros::BoundingBoxes& msg);
    
  // Subscribe Lidar Topic
  void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

private:
  
  // ROS Variables
  ros::NodeHandle nh_;
  ros::Subscriber sub_image_;
  ros::Subscriber sub_yolo_;
  ros::Subscriber sub_lidar_;
  std::string sub_image_topic_name_;
  std::string sub_yolo_topic_name_;
  std::string sub_lidar_topic_name_;
  int queue_size_;

  // OpenCV Image processing Variables
  cv::Mat frame_;
  cv::Mat yolo_result_;
  cv::Mat homography_;
  cv::Mat homography_matrix_;

  // For Traffic sign
  bool is_traffic_light_ = false;
  bool is_red_ = false;
  bool is_green_ = false;
  bool is_yellow_ = false;

  // For Traffic sign
  float traffic_sign_detection_threshold_;
  bool is_left_sign_ = false;
  bool is_right_sign_ = false;
  bool is_stop_sign_ = false;
  bool is_crosswalk_sign_ = false;

  // Debug Flag
  bool debug_;

  // Yolo-v3 tiny image size
  int yolo_width_;
  int yolo_height_;
  float object_width_;
  int image_width_;
  int image_height_;

  // Calibration
  int calibration_mode_; // 0: Normal, 1: fisheye 2: Not Calibration
  bool is_first_calibration_ = true;
  cv::Mat mapx_, mapy_;

  // Lidar
  CLidar CLidar_;
    
  // Yolo object
  std::vector<DetectionObject> DetectionObjects_;
  
};


#endif  // CAMER_LIDAR_FUSION_H_
