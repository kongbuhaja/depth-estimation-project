/**
 * @file main.cpp
 * @author Jongrok Lee (lrrghdrh@naver.com)
 * @brief Lane Keeping System Main Function using Hough Transform
 * @version 0.2
 * @date 2022-11-27
 */
#include "traffic_sign_depth_estimation/camera_lidar_fusion.h"

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "Traffic Sign Depth Estimation");
  std::cout << "TSDE Start" << std::endl;
  CameraLidarFusion CLF;
  CLF.run();

  return 0;
}