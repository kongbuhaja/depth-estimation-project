/**
 * @file camera_depth_estimation.h
 * @author Seungho Hyeong (slkumquat@gmail.com)
 * @brief Camera Depth Estimation class header file
 * @version 1.0
 * @date 2023-02-15
 */
#ifndef CAMERA_DEPTH_ESTIMATION_H_
#define CAMERA_DEPTH_ESTIMATION_H_
#include <yaml-cpp/yaml.h>

#include <iostream>
#include <string>
#include <vector>
#include <cmath>

#include "opencv2/opencv.hpp"

/*
  #define image_width 640
  #define D_PI 180
  #define PI M_PI
  #define OBJECT_WIDTH 0.195
  
  class BBox
  {
    public:
        BBox(const float xmin_, const float ymin_, const float xmax_, const float ymax_);
        
        float xmin;
        float ymin;
        float xmax;
        float ymax;
  };

  class CCamera
  {
    public:
      CCamera();
      ~CCamera();

      float get_focal_length();

      float get_distance(const float object_width_in_image);

      void data_init(std::vector<BBox> &Bboxes);

      void get_ad();

      std::vector<std::vector<float>> get_camera_ad();
      
      std::vector<std::vector<float>> result;

    private:
      cv::Mat image;
      std::vector<BBox> m_boxes;
      float focal_length = 317.33333333;
      float fov = 211;
      // std::vector<std::vector<float>> result;    
    };
*/

#endif  // CAMERA_DEPTH_ESTIMATION_H_
