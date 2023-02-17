/**
 * @file lidar_depth_estimation.cpp
 * @author Seungho Hyeong (slkumquat@gmail.com)
 * @brief Lidar Depth Estimation class source file
 * @version 1.0
 * @date 2023-02-15
 */
#include "traffic_sign_depth_estimation/lidar_depth_estimation.h"



CLidar::CLidar( void )
{
}

CLidar::~CLidar( void )
{
}

void CLidar::CL_Data_init(float ranges[])
{
    for (int i = 0; i < LIDAR_RAY_CNT; i++)
    {
        m_fLidarDistance[i] = ranges[i];
    }
}

bool CLidar::CL_Init()
{
	int i;

	for ( i = 0; i < LIDAR_RAY_CNT; i++ )
	{
		m_fSin[i] = sin( ((LIDAR_ANGLE_MAX * 2) / LIDAR_RAY_CNT) * i );
		m_fCos[i] = cos( ((LIDAR_ANGLE_MAX * 2) / LIDAR_RAY_CNT) * i );
	}


	return true;
}

bool CLidar::CL_Process_Distance( float fOffset_X, float fOffset_Y )
{
	int i;


	m_fOffset_X = fOffset_X;
	m_fOffset_Y = fOffset_Y;

	for ( i = 0; i < LIDAR_RAY_CNT; i++ )
	{
		// 입력 데이터 처리
		m_DistanceXY_Origin[i].X = m_fSin[i] * m_fLidarDistance[i];		// 가로
		m_DistanceXY_Origin[i].Y = m_fCos[i] * m_fLidarDistance[i];		// 거리
		m_DistanceXY_Origin[i].fDistance = m_fLidarDistance[i];
		m_DistanceXY_Origin[i].nClusteringNo = -1;


		// OFFSET 만큼 이동(좌표 이동)
		m_DistanceXY_Offset[i].X = m_DistanceXY_Origin[i].X + m_fOffset_X;
		m_DistanceXY_Offset[i].Y = m_DistanceXY_Origin[i].Y + m_fOffset_Y;
		m_DistanceXY_Offset[i].fDistance = sqrt( (m_DistanceXY_Offset[i].X * m_DistanceXY_Offset[i].X) + (m_DistanceXY_Offset[i].Y * m_DistanceXY_Offset[i].Y) );
		m_DistanceXY_Offset[i].nClusteringNo = -1;
	}


	return true;
}


bool CLidar::CL_Process_DBScan( int nMinPts, float fEpsilon, int nMaxScanPts )
{
	LADAR_DistanceXY *pCenterPts;
	LADAR_DistanceXY *pComparePts;
	float fDistancePts;
	int		nClusteringCenterPts;
	int		nClusteringNo;
	int		nClusteringCnt;
	int		nClusteringNo_Next;
	int		nClusteringNo_Center;
	int i;
	int j;


	m_DBScan_MinPts				= nMinPts;
	m_DBScan_Epsilon			= fEpsilon;
	m_DBScan_Epsilon_MaxScanPts = nMaxScanPts;




	// 계산의 편의를 위해, 딱 필요한 만큼의 데이터를 m_DistanceXY_DBScan 으로 옮겨준다.
	// 단, (LIDAR_DBSCAN_RAY_CNT / 2) 가 기존의 정중앙값이다.
	m_DistanceXY_DBScan[(LIDAR_DBSCAN_RAY_CNT / 2)] = m_DistanceXY_Offset[0];

	for ( i = 1; i <= (LIDAR_DBSCAN_RAY_CNT / 2); i++ )
	{
		m_DistanceXY_DBScan[i + (LIDAR_DBSCAN_RAY_CNT / 2)] = m_DistanceXY_Offset[i];
	}
	for ( i = LIDAR_RAY_CNT - (LIDAR_DBSCAN_RAY_CNT / 2) ; i < LIDAR_RAY_CNT; i++ )
	{
		m_DistanceXY_DBScan[i - (LIDAR_RAY_CNT - (LIDAR_DBSCAN_RAY_CNT / 2))] = m_DistanceXY_Offset[i];
	}


	nClusteringNo_Next = 0;

	for ( i = (m_DBScan_Epsilon_MaxScanPts / 2); i < LIDAR_DBSCAN_RAY_CNT - (m_DBScan_Epsilon_MaxScanPts / 2); i++ )
	{
		nClusteringCnt = 0;
		pCenterPts = &m_DistanceXY_DBScan[i + (m_DBScan_Epsilon_MaxScanPts / 2)];
		for ( j = i; j < i + m_DBScan_Epsilon_MaxScanPts; j++ )
		{
			pComparePts = &m_DistanceXY_DBScan[j];
			if ( pCenterPts == pComparePts )
				continue;

			fDistancePts = sqrt( ((pCenterPts->X - pComparePts->X) * (pCenterPts->X - pComparePts->X)) + ((pCenterPts->Y - pComparePts->Y) * (pCenterPts->Y - pComparePts->Y)) );
			if ( fDistancePts <= m_DBScan_Epsilon )
			{
				nClusteringCnt++;
			}
		}

		if ( nClusteringCnt >= m_DBScan_MinPts )
		{
			if ( pCenterPts->nClusteringNo < 0 )
			{
				pCenterPts->nClusteringNo = nClusteringNo_Next;
				nClusteringNo_Next++;
			}

			nClusteringNo_Center = pCenterPts->nClusteringNo;
			for ( j = i; j < i + m_DBScan_Epsilon_MaxScanPts; j++ )
			{
				pComparePts = &m_DistanceXY_DBScan[j];
				if ( pCenterPts == pComparePts )
					continue;

				fDistancePts = sqrt( ((pCenterPts->X - pComparePts->X) * (pCenterPts->X - pComparePts->X)) + ((pCenterPts->Y - pComparePts->Y) * (pCenterPts->Y - pComparePts->Y)) );
				if ( (fDistancePts <= m_DBScan_Epsilon) && (pComparePts->nClusteringNo < 0) )
				{
					pComparePts->nClusteringNo = nClusteringNo_Center;
				}
			}
		}
	}


	m_DistanceXYClusterAvgCnt = nClusteringNo_Next;
	//printf("%d\n", m_DistanceXYClusterAvgCnt );
	for ( i = 0; i < LIDAR_DBSCAN_RAY_CNT; i++ )
	{
		m_DistanceXYClusterAvg[i].Cluster.X = 0;
		m_DistanceXYClusterAvg[i].Cluster.Y = 0;
		m_DistanceXYClusterAvg[i].Cluster.fDistance= 0;
		m_DistanceXYClusterAvg[i].Cluster.nClusteringNo = -1;
		m_DistanceXYClusterAvg[i].fAngle = 0;
		m_DistanceXYClusterAvg[i].nPtsCnt = 0;
		m_DistanceXYClusterAvg[i].nClusteringStartIndex = -1;
	}

	for ( i = 0; i < LIDAR_DBSCAN_RAY_CNT; i++ )
	{
		nClusteringNo = m_DistanceXY_DBScan[i].nClusteringNo;
		if ( nClusteringNo >= 0 )
		{
			m_DistanceXYClusterAvg[nClusteringNo].Cluster.X += m_DistanceXY_DBScan[i].X;
			m_DistanceXYClusterAvg[nClusteringNo].Cluster.Y += m_DistanceXY_DBScan[i].Y;
			m_DistanceXYClusterAvg[nClusteringNo].Cluster.fDistance += m_DistanceXY_DBScan[i].fDistance;
			m_DistanceXYClusterAvg[nClusteringNo].Cluster.nClusteringNo = m_DistanceXY_DBScan[i].nClusteringNo;
			m_DistanceXYClusterAvg[nClusteringNo].nPtsCnt++;
			if ( m_DistanceXYClusterAvg[nClusteringNo].nClusteringStartIndex < 0 )
				m_DistanceXYClusterAvg[nClusteringNo].nClusteringStartIndex = i;
		}
	}


	for ( i = 0; i < m_DistanceXYClusterAvgCnt; i++ )
	{
		m_DistanceXYClusterAvg[i].Cluster.X = m_DistanceXYClusterAvg[i].Cluster.X / m_DistanceXYClusterAvg[i].nPtsCnt;
		m_DistanceXYClusterAvg[i].Cluster.Y = m_DistanceXYClusterAvg[i].Cluster.Y / m_DistanceXYClusterAvg[i].nPtsCnt;
		m_DistanceXYClusterAvg[i].Cluster.fDistance = m_DistanceXYClusterAvg[i].Cluster.fDistance / m_DistanceXYClusterAvg[i].nPtsCnt;

		nClusteringCenterPts = ((m_DistanceXYClusterAvg[i].nClusteringStartIndex * 2) + m_DistanceXYClusterAvg[i].nPtsCnt) / 2;

		if ( nClusteringCenterPts == (int)(LIDAR_DBSCAN_RAY_CNT / 2) )
			m_DistanceXYClusterAvg[i].fAngle = 0;
		else if ( nClusteringCenterPts < (int)(LIDAR_DBSCAN_RAY_CNT / 2) )
		{
			m_DistanceXYClusterAvg[i].fAngle = LIDAR_ANGLE_INC * ((LIDAR_DBSCAN_RAY_CNT / 2) - nClusteringCenterPts);
		}
		else
		{
			m_DistanceXYClusterAvg[i].fAngle = LIDAR_ANGLE_INC * (nClusteringCenterPts - (LIDAR_DBSCAN_RAY_CNT / 2) );
		}

		// printf("%d) %f, %f, %f, %f\n", m_DistanceXYClusterAvg[i].Cluster.nClusteringNo, m_DistanceXYClusterAvg[i].Cluster.X, m_DistanceXYClusterAvg[i].Cluster.Y, m_DistanceXYClusterAvg[i].Cluster.fDistance, m_DistanceXYClusterAvg[i].fAngle );
	}

    cv::Mat	Image = cv::Mat(450, 540, CV_8UC3, cv::Scalar(0, 0, 0));
    m_pCameraImg = &Image;

	for ( i = 0; i < LIDAR_DBSCAN_RAY_CNT; i++ )
	{
		int X, Y;

		X = ((int)(m_DistanceXY_DBScan[i].X * 100))*2 + 270;
		Y = (-(int)(m_DistanceXY_DBScan[i].Y * 100))*2 + 449;

        cv::circle(*m_pCameraImg, cv::Point(X, Y), 2, cv::Scalar::all(m_DistanceXY_DBScan[i].nClusteringNo*15), -1, 8, 0);
	}
	for ( i = 0; i < m_DistanceXYClusterAvgCnt; i++ )
	{
		char buf[128];

		sprintf( buf, "x: %.2f, y: %.2f%c", m_DistanceXYClusterAvg[i].Cluster.X, m_DistanceXYClusterAvg[i].Cluster.Y, '\0');
		
		
		int X = ((int)(m_DistanceXYClusterAvg[i].Cluster.X * 100))*2 + 180;
		int Y = (-(int)(m_DistanceXYClusterAvg[i].Cluster.Y * 100))*2 + 449;
		cv::putText(*m_pCameraImg, buf, cv::Point(X, Y), 2, 0.5, cv::Scalar(255, 0, 0), 1);
	}
    //
    for (int i = 1; i < 5; i++)
    {
        cv::line(*m_pCameraImg, cv::Point(0, 90*i), cv::Point(540, 90*i), cv::Scalar(50, 50, 50), 1, 8, 0);
    }
    //
    for (int i = 1; i < 6; i++)
    {
        cv::line(*m_pCameraImg, cv::Point(90*i, 0), cv::Point(90*i, 450), cv::Scalar(50, 50, 50), 1, 8, 0);
    }
	cv::imshow("Lidar Detection", *m_pCameraImg);
	cv::waitKey(1);

	return true;
}

void CLidar::CL_MatchingPoint(float image_x, float image_y, float *lidar_x, float *lidar_y)
{
	float min_distance = 100000;
	int index = -1;
	for ( int i = 0; i < m_DistanceXYClusterAvgCnt; i++ )
	{
		float distance = ((-m_DistanceXYClusterAvg[i].Cluster.X - image_x) * (-m_DistanceXYClusterAvg[i].Cluster.X - image_x))
			+ ((m_DistanceXYClusterAvg[i].Cluster.Y - image_y) * (m_DistanceXYClusterAvg[i].Cluster.Y - image_y));

		if (min_distance > distance)
		{
			if (m_DistanceXYClusterAvg[i].Cluster.nClusteringNo > -1)
			{
				min_distance = distance;
				index = i;
			}
		}

	}
	// m_DistanceXY_DBScan[index].nClusteringNo  = -1;
	*lidar_x = m_DistanceXYClusterAvg[index].Cluster.X;
	*lidar_y = m_DistanceXYClusterAvg[index].Cluster.Y;
}