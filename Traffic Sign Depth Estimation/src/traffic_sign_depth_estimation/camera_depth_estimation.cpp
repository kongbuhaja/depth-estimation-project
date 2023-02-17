/**
 * @file hough_transform_lane_detector.cpp
 * @author Seungho Hyeong (slkumquat@gmail.com)
 * @brief hough transform lane detector class source file
 * @version 1.0
 * @date 2023-01-19
 */
#include "traffic_sign_depth_estimation/camera_depth_estimation.h"


/*
    BBox::BBox(const float xmin_, const float ymin_, const float xmax_, const float ymax_){
        xmin = xmin_;
        ymin = ymin_;
        xmax = xmax_;
        ymax = ymax_;
    }

    CCamera::CCamera()
    {
    }
    CCamera::~CCamera()
    {
    }
    float CCamera::get_focal_length()
    {
        float camera_to_origin = 0.145;
        float real_distance = 0.45 + camera_to_origin;
        float test_object_width = 0.03;
        float focal_length = (test_object_width * real_distance) / test_object_width;
        return focal_length;
    }

    float CCamera::get_distance(const float object_width_in_image){
        float distance = (OBJECT_WIDTH * focal_length) / object_width_in_image;
        return distance;
    }

    void CCamera::data_init(std::vector<BBox> &Bboxes)
    {
        m_boxes.clear();
        std::cout << Bboxes.size() << std::endl;
        for (int i = 0; Bboxes.size(); i++)
        {
            m_boxes.push_back(Bboxes[i]);
        }

    }

    void CCamera::get_ad()
    {
        result.reserve(m_boxes.size());
        for (auto bbox : m_boxes){
            float distance = get_distance(bbox.xmax-bbox.xmin);
            float azimuth = ((bbox.xmax+bbox.xmin)/2 - image_width/2) / image_width * fov / 2;
            float radian = azimuth / D_PI * PI;
            result.push_back({distance * sin(radian), distance * cos(radian)});
        }
    }

    std::vector<std::vector<float>> CCamera::get_camera_ad()
    {
        return result;
    }
    */

