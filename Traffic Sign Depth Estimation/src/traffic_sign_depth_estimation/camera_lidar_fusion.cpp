/**
 * @file camera_lidar_fusion.cpp
 * @author Seungho Hyeong (slkumquat@gmail.com)
 * @brief Camera Lidar Fusion Class source file
 * @version 1.0
 * @date 2023-02-15
 */
#include "traffic_sign_depth_estimation/camera_lidar_fusion.h"


CameraLidarFusion::CameraLidarFusion() {
  std::cout << "CLF construct" << std::endl;
  std::string config_path = "/home/nvidia/xycar_ws/src/YouOnlyLiveOnce/Traffic Sign Depth Estimation/Traffic Sign Depth Estimation/config/config.yaml";
  YAML::Node config = YAML::LoadFile(config_path);

  std::cout << "Set Param" << std::endl;
  setParams(config);

  sub_image_ = nh_.subscribe(
    sub_image_topic_name_, queue_size_, &CameraLidarFusion::imageCallback, this);
  sub_yolo_ = nh_.subscribe(
    sub_yolo_topic_name_, queue_size_, &CameraLidarFusion::yoloCallback, this);
  sub_lidar_ = nh_.subscribe(
    sub_lidar_topic_name_, queue_size_, &CameraLidarFusion::lidarCallback, this);
}

void CameraLidarFusion::setParams(const YAML::Node &config) {
  sub_image_topic_name_ = config["TOPIC"]["SUB_IMAGE_NAME"].as<std::string>();
  sub_yolo_topic_name_ = config["TOPIC"]["SUB_YOLO_NAME"].as<std::string>();
  sub_lidar_topic_name_ = config["TOPIC"]["SUB_LIDAR_NAME"].as<std::string>();
  queue_size_ = config["TOPIC"]["QUEUE_SIZE"].as<int>();
    
  yolo_width_ = config["YOLO"]["WIDTH"].as<int>();
  yolo_height_ = config["YOLO"]["HEIGHT"].as<int>();
  object_width_ = config["YOLO"]["OBJECT_WIDTH"].as<float>();

  image_width_ =  config["IMAGE"]["WIDTH"].as<int>();
  image_height_ = config["IMAGE"]["HEIGHT"].as<int>();    
  calibration_mode_ = config["IMAGE"]["CALIBRATION_MODE"].as<int>();
  
  debug_ = config["DEBUG"].as<bool>();
}

CameraLidarFusion::~CameraLidarFusion() {
  std::cout << "Program End" << std::endl;
}

void CameraLidarFusion::run() {
  std::cout << "CLF run" << std::endl;
  ros::Rate rate(30);
  
  while (ros::ok()) {
    ros::spinOnce();
    if (frame_.empty()) {
      continue;
    }
    // Begin time.
    auto time_ = ros::Time::now().toSec();

    

  }
}

void CameraLidarFusion::imageCallback(const sensor_msgs::Image &msg) {
  //std::cout << "image callback" << std::endl;
  cv::Mat src = cv::Mat(msg.height,
                        msg.width,
                        CV_8UC3,
                        const_cast<uint8_t *>(&msg.data[0]),
                        msg.step);

  // Calibration
  cv::Mat camera_materix, dist_coeffs;
  if (calibration_mode_ == 0)
  {
    cv::Mat frame;
    cv::cvtColor(src, frame, cv::COLOR_RGB2BGR);
    
    cv::Mat camera_matrix = cv::Mat::eye(3, 3, CV_64FC1);
    cv::Mat dist_coeffs = cv::Mat::zeros(1, 5, CV_64FC1);

    camera_matrix = (cv::Mat1d(3, 3) << 775.95650, 0., 105.98641, 0., 923,72803, 230.30234, 0., 0., 1.);
    dist_coeffs = (cv::Mat1d(1, 5) << -1.65346, 0.29905, -0.01189, 0.35797, 0.14341);
    if(is_first_calibration_ == true)
    {
      is_first_calibration_ = false;

      cv::initUndistortRectifyMap(camera_matrix,
                                  dist_coeffs,
                                  cv::Mat(),
                                  camera_matrix,
                                  frame.size(),
                                  CV_32FC1,
                                  mapx_,
                                  mapy_);
    }
    frame_ = frame.clone();
    cv::remap(frame, frame_, mapx_, mapy_, cv::INTER_LINEAR);
  }
  else if (calibration_mode_ == 1)
  {
    cv::Mat frame;
    cv::cvtColor(src, frame, cv::COLOR_RGB2BGR);
    
    cv::Mat camera_matrix = cv::Mat::eye(3, 3, CV_64FC1);
    cv::Mat dist_coeffs = cv::Mat::zeros(1, 5, CV_64FC1);
    cv::Mat optimal_new_camera_matrix = cv::Mat::eye(3, 3, CV_64FC1);

    camera_matrix = (cv::Mat1d(3, 3) << 363.55961, 0., 330.71213, 0., 364.02549, 232.83067, 0., 0., 1.);
    dist_coeffs = (cv::Mat1d(1, 4) << -0.04390, -0.03618, 0.79952, -0.05081);
    optimal_new_camera_matrix = (cv::Mat1d(3, 3) << 300.94705, 0., 339.31511, 0., 301.33269, 230.39529, 0., 0., 1.);
    //optimal_new_camera_matrix = (cv::Mat1d(3, 3) << 238.83961, 0., 335.32899, 0., 239.14567, 232.37744, 0., 0., 1.);
    if(is_first_calibration_ == true)
    {
      is_first_calibration_ = false;
      cv::Mat E = cv::Mat::eye(3, 3, cv::DataType<double>::type);
      cv::fisheye::initUndistortRectifyMap(camera_matrix, dist_coeffs,
        E, camera_matrix,
        cv::Size(frame.cols, frame.rows), CV_32FC1, mapx_, mapy_);
    }
    
    cv::remap(frame, frame_, mapx_, mapy_, cv::INTER_LINEAR);
  }
  else
  {
    cv::cvtColor(src, frame_, cv::COLOR_RGB2BGR);
  }
  frame_.copyTo(yolo_result_);

  homography_matrix_ = cv::Mat::eye(3, 3, CV_64FC1);  
  homography_matrix_ = (cv::Mat1d(3, 3) << -1.41632e-01, -8.91008e-01, 2.51905e+02, 2.90843e-02, -2.36186e+00, 5.90243e+02, 5.20600e-05, -4.39864e-03, 1.00000e+00);
  
  // cv::warpPerspective(frame_, homography_, homography_matrix_, cv::Size(640, 480));
  // cv::imshow("homography", homography_);
  // cv::waitKey(1);
}

void CameraLidarFusion::lidarCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
  //std::cout << "lidar callback" << std::endl;
  float ranges[505];

  for (int i = 0; i < 505; i++)
  {
    ranges[i] = msg->ranges[i];
  }
  
  CLidar_.CL_Data_init(ranges);
  CLidar_.CL_Init();
  CLidar_.CL_Process_Distance( 0, -0.043 );
	CLidar_.CL_Process_DBScan( 3, 0.3, 10);
}

void CameraLidarFusion::yoloCallback(const yolov3_trt_ros::BoundingBoxes& msg) {
  //std::cout << "yolo callback" << std::endl;
  DetectionObjects_.clear();
  DetectionObject object;
  cv::Mat	Result_Image = cv::Mat(450, 540, CV_8UC3, cv::Scalar(0, 0, 0));

  static cv::Scalar Blue(255, 0, 0); // 0
  static cv::Scalar Yellow(0, 255, 255); // 1
  static cv::Scalar Red(0, 0, 255); // 2
  static cv::Scalar Green(0, 255, 0); // 3

  cv::Scalar color;

  for (int i = 1; i < 5; i++)
  {
      cv::line(Result_Image, cv::Point(0, 90*i), cv::Point(540, 90*i), cv::Scalar(50, 50, 50), 1, 8, 0);
  }
  for (int i = 1; i < 6; i++)
  {
      cv::line(Result_Image, cv::Point(90*i, 0), cv::Point(90*i, 450), cv::Scalar(50, 50, 50), 1, 8, 0);
  }

  for (auto& box : msg.bounding_boxes)
  {
    
    object.id = box.id;
    object.probability = box.probability;
    object.xmax = static_cast<int>(static_cast<float>(box.xmax) * (640. / (float)yolo_width_));
    object.ymax = static_cast<int>(static_cast<float>(box.ymax) * (480. / (float)yolo_height_));
    object.xmin = static_cast<int>(static_cast<float>(box.xmin) * (640. / (float)yolo_width_));
    object.ymin = static_cast<int>(static_cast<float>(box.ymin) * (480. / (float)yolo_height_));
    int sign_area = (box.xmax - box.xmin) * (box.ymax - box.ymin);
    object.area = sign_area;

    if (object.xmin < 0) object.xmin = 0;
    if (object.ymin < 0) object.ymin = 0;
    if (object.xmax > 640) object.xmax = 640 - 1;
    if (object.ymax > 480) object.ymax = 480 - 1;

    if (object.id == 0)
    {
      color = Blue;
    }
    else if (object.id == 1)
    {
      color = Yellow;
    }
    else if (object.id == 2)
    {
      color = Red;
    }
    else if (object.id == 3)
    {
      color = Green;
    }
    
    cv::rectangle(yolo_result_, cv::Rect(cv::Point(object.xmin, object.ymin), cv::Point(object.xmax, object.ymax)),
      color, 2, 8, 0);
    
    float focal_length = 317.33333333;
    float fov = 211.;

    float distance = ((float)object_width_ * focal_length) / (float)(object.xmax - object.xmin);
    float azimuth = ((float)(object.xmax + object.xmin)/2. - (float)image_width_/2.) / (float)image_width_ * fov / 2.;
    double PI = 3.14159265;
    float radian = azimuth * (PI / 180.);
    
    char buf[128];

		sprintf(buf, "x: %.2f, y: %.2f%c", distance * sin(radian), distance * cos(radian), '\0');
    //std::string text = "x: " + std::to_string(distance * sin(radian)) + ", y: " + std::to_string(distance * cos(radian));
    cv::putText(yolo_result_, buf, cv::Point(object.xmin, object.ymin), 2, 0.5, color, 1);

    float lidar_x, lidar_y;
    CLidar_.CL_MatchingPoint(distance * sin(radian), distance * cos(radian),  &lidar_x, &lidar_y);

    

		sprintf( buf, "x: %.2f, y: %.2f%c", lidar_x, lidar_y, '\0');
		
		
		int X = ((int)(lidar_x * 100))*2 + 270;
		int Y = (-(int)(lidar_y * 100))*2 + 449;
    cv::circle(Result_Image, cv::Point(X, Y), 10, color, -1, 8, 0);
		cv::putText(Result_Image, buf, cv::Point(X, Y), 2, 0.5, color, 1);
    
    
    DetectionObjects_.push_back(object);
  } 
  cv::imshow("yolo result", yolo_result_);
  cv::imshow("detection result", Result_Image);
  cv::waitKey(1);

  
  
  /*
	for ( i = 0; i < LIDAR_DBSCAN_RAY_CNT; i++ )
	{
		int X, Y;

		X = (-(int)(m_DistanceXY_DBScan[i].X * 100))*2 + 180;
		Y = (-(int)(m_DistanceXY_DBScan[i].Y * 100))*2 + 449;

        cv::circle(*m_pCameraImg, cv::Point(X, Y), 2, cv::Scalar::all(m_DistanceXY_DBScan[i].nClusteringNo*15), -1, 8, 0);
	}
	for ( i = 0; i < m_DistanceXYClusterAvgCnt; i++ )
	{
		char buf[128];

		sprintf( buf, "x: %.2f, y: %.2f%c", m_DistanceXYClusterAvg[i].Cluster.X, m_DistanceXYClusterAvg[i].Cluster.Y, '\0');
		
		
		int X = (-(int)(m_DistanceXYClusterAvg[i].Cluster.X * 100))*2 + 180;
		int Y = (-(int)(m_DistanceXYClusterAvg[i].Cluster.Y * 100))*2 + 449;
		cv::putText(*m_pCameraImg, buf, cv::Point(X, Y), 2, 0.5, cv::Scalar(255, 0, 0), 1);
	}
    //
    for (int i = 1; i < 5; i++)
    {
        cv::line(*m_pCameraImg, cv::Point(0, 90*i), cv::Point(360, 90*i), cv::Scalar(50, 50, 50), 1, 8, 0);
    }
    //
    for (int i = 1; i < 4; i++)
    {
        cv::line(*m_pCameraImg, cv::Point(90*i, 0), cv::Point(90*i, 450), cv::Scalar(50, 50, 50), 1, 8, 0);
    }
	cv::imshow("camera", *m_pCameraImg);
  */
}


