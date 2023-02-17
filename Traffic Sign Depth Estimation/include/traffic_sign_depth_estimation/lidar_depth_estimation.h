/**
 * @file lidar_depth_estimation.h
 * @author Seungho Hyeong (slkumquat@gmail.com)
 * @brief Lidar Depth Estimation class header file
 * @version 1.0
 * @date 2023-02-15
 */
#ifndef LIDAR_DEPTH_ESTIMATION_H_
#define LIDAR_DEPTH_ESTIMATION_H_
#include <yaml-cpp/yaml.h>

#include <iostream>
#include <string>
#include <vector>

#include "opencv2/opencv.hpp"


// Lidar information
#define LIDAR_RAY_CNT		505
#define LIDAR_ANGLE_MIN		-3.14159274101
#define LIDAR_ANGLE_MAX		3.14159274101
#define LIDAR_ANGLE_INC		(float)((LIDAR_ANGLE_MAX * 2) / (float)LIDAR_RAY_CNT)

#define LIDAR_DBSCAN_RAY_CNT		261
struct LADAR_DistanceXY
{
	float	X;
	float	Y;
	float	fDistance;
	int		nClusteringNo;
};

struct LADAR_DistanceXYClusterAvg
{
	LADAR_DistanceXY Cluster;
	float	fAngle;
	int		nPtsCnt;
	int		nClusteringStartIndex;
};


class CLidar
{
private:
	
	float				m_fOffset_X;
	float				m_fOffset_Y;

	int					m_DBScan_MinPts;
	float				m_DBScan_Epsilon;
	int					m_DBScan_Epsilon_MaxScanPts;

	float				m_fSin[LIDAR_RAY_CNT];
	float				m_fCos[LIDAR_RAY_CNT];
	LADAR_DistanceXY	m_DistanceXY_Origin[LIDAR_RAY_CNT];
	LADAR_DistanceXY	m_DistanceXY_Offset[LIDAR_RAY_CNT];
	LADAR_DistanceXY	m_DistanceXY_DBScan[LIDAR_DBSCAN_RAY_CNT];
	float				m_fLidarDistance[LIDAR_RAY_CNT];

	LADAR_DistanceXYClusterAvg	m_DistanceXYClusterAvg[LIDAR_DBSCAN_RAY_CNT];
	int							m_DistanceXYClusterAvgCnt;
	
	cv::Mat				*m_pCameraImg;
	


public:
	CLidar( void );
	~CLidar( void );

	bool		CL_Init();
    void        CL_Data_init(float ranges[]);


	bool		CL_Process_Distance( float fOffset_X, float fOffset_Y );
	bool		CL_Process_DBScan( int nMinPts, float fEpsilon, int nMaxScanPts );

	void		CL_MatchingPoint(float image_x, float image_y, float *lidar_x, float *lidar_y);
};

#endif  // LIDAR_DEPTH_ESTIMATION_H_
