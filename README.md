# Depth Estimation Project - YouOnlyLiveOnce

## Project Description
- 탁구공 거리 추정 프로젝트 
  - xycar에 장착된 usb-cam을 사용하여 탁구공을 검출한 뒤, 거리를 추정
- 교통 표지판 거리 추정 프로젝트
  - xycar에 장착된 usb-cam을 사용하여 교통 표지판을 검출한 뒤, 거리를 추정
  - 또한, 2D-LiDAR를 사용하여 카메라와 Sensor Fusion을 진행한 뒤, 보다 정확한 거리 추정

# 탁구공 거리 추정 프로젝트
## Team Introduction
- 안현석: model 학습, camera calibration, depth 추정
- 장명근: ROS, C++ 적용
- 진창용: data 수집 및 labeling, model 변환
- 형승호: ROS, C++ 적용

## Object Detection
- YOLO_v3-tiny model 사용
- flip_horiziontal, sharpen, random_multiply brightness
- Kmeans 알고리즘을 통한 anchor 최적화
- 작은 object detection을 위한 input_size 변경(608x608)
- 학습 과정 및 결과
  - 탁구공 단일 데이터 수집 및 labeling  
![0_0](https://user-images.githubusercontent.com/42567320/219880867-a2a83d94-cf43-4005-9a8e-6e6f87ff426e.png)
![1_0](https://user-images.githubusercontent.com/42567320/219880868-3bb3e545-4934-45b9-9dc2-392bfb81a366.png)

## Camera Calibration
- checkerboard를 활용한 Intrinsic matrix, distCoeffs 추출
- 추출한 Intrinsic matrix, distCoeffs 기반 camera calibration 실행
- 카메라의 특성에 맞게 OpenCV의 Fisheye 모델을 적용
- https://docs.opencv.org/3.4/db/d58/group__calib3d__fisheye.html 
- checkerboard를 사용하여 코너 검출 결과

![checkerboard](https://github.com/prgrms-ad-devcourse/ad-4-depth-estimation-project/blob/YouOnlyLiveOnce_team5/Result%20Image/checkerboard.PNG)

![Calibration Result](https://github.com/prgrms-ad-devcourse/ad-4-depth-estimation-project/blob/YouOnlyLiveOnce_team5/Result%20Image/Calibration%20result.PNG)



## Distance Estimation
- homography 기반 BEV 변환 후 depth 추정
- homography 기법은 바닥 위치 기준임으로 object cx, cy-h/2 지점 사용  
![image](https://user-images.githubusercontent.com/42567320/219881364-abc59ff2-5532-40d4-baeb-77de0244f6aa.png)
![image](https://user-images.githubusercontent.com/42567320/219881383-32edb747-ae53-4ce7-b287-6315f665d68e.png)
- Eestimation 결과  
[[-56.594981   122.64750137]            [[ -3.7579628   84.39510498]
 [ 48.29775238  39.60304871]             [ 47.89889374 131.38976212]
 [ 51.63368225 199.19552021]]            [-51.90259666  36.40488739]]

# 교통 표지판 거리 추정 프로젝트
## Team Introduction
- 안현석: model 학습, camera calibration, camera 기반 depth 추정, Extrinsic calibration
- 장명근: ROS, C++ 적용
- 진창용: data 수집 및 labeling, model 변환, lidar 기반 depth 추정, lidar points clustering
- 형승호: Extrinsic calibration, lidar 및 camera objects matching, ROS, C++ 적용

## Object Detection
- YOLO_v3-tiny model 사용
- flip_horiziontal, sharpen, random_multiply brightness
- Kmeans 알고리즘을 통한 anchor 최적화
- 작은 object detection을 위한 input_size 변경(608x608)
- 학습 과정 및 결과
  - 기존 TSTL 주행간 사용한 데이터 camera calibration 후 

## Distance Estimation
- Camera
  - object real width, object pixel width 기반 focal_length 추출
  - camera calibration 후 반복을 통해 FOV 설정
  - focal_length, FOV, object pixcel width 기반 depth 추출  
  
![image](https://user-images.githubusercontent.com/42567320/219882392-95a6329a-8964-41a0-8a78-6a12b6e5ac1f.png)

- LiDAR
  - 카메라와 비슷한 FOV를 가지기 위해 -90 ~ 90 구간을 ROI로 설정
  - 측정된 LiDAR 데이터를 DBSCAN 기법을 사용하여 Object Clustering을 진행
  - 아래의 사진은 Clustering된 데이터들의 평균 x와 평균 y값을 출력한 것임.
  - 아래 사진에서 한 칸의 크기는 45cm로 표시하였음.
  
![image](https://user-images.githubusercontent.com/42567320/219882988-df0574ee-e68a-4c99-ad8a-0abc136b5fb0.png)

- Camera, LiDAR의 관측된 objects matching
  - 카메라와 라이다에서 각각 측정된 object간 매칭은, 거리를 기반으로 하여 가장 가까운 object를 같은 obejct로 인식.
  - 아래 사진은 objects matching을 한 결과로, 빨간색은 정지, 노란색은 우회전, 파란색은 좌회전, 초록색은 횡단보도의 ID를 의미한다.
  
![image](https://user-images.githubusercontent.com/42567320/219882969-57cedd19-8c90-49f1-9346-fc4a59d6d265.png)


## Lesson Learn (정답은 없습니다. 자유롭게 생각해보세요.)
- Camera 기반의 거리추정 방법의 장/단점
  - 장점
    - 사람의 눈과 동일하게 측정되기 때문에 시각화에 다른 작업이 필요하지 않음.
    - 넓은 수직 시야각을 통해 어느 정도 가려져 있는 물체도 인식 가능.
  - 단점
    - LiDAR에 비하면 측정된 Depth가 정확하지 않음
    - 사용하기 전 보정이 필수적이며, 먼 거리의 물체는 측정하지 힘듬.
- LiDAR 기반의 거리추정 방법의 장/단점
  - 장점
    -  빛의 이동 거리에 따른 시간을 측정하는 방식으로, 정확한 거리 값을 측정함.
    -  카메라와 다르게, 어둡거나 너무 밝은 환경에서 동작함.
  - 단점
    -  가려져 있는 물체는 인식하지 못함.
