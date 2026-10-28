# 장애물 회피 제외 통합 시험본

이 버전은 ROS Noetic / MORAI에서 현장 검증하기 위한 통합 소스다. ROS 실행·catkin 빌드·실제 MORAI 주행은 제작 환경에서 수행하지 못했다.

## 기준
- AutoVehicle jihun_ws 작업공간의 센서/통신/곡률 및 신호 처리.
- 사용자 첨부 고속도로 Python과 launch의 고속도로 파라미터.
- roundabout_v6_1의 판단 및 제어. validation/reference_path.py와 reference_longitudinal_controller.py를 전용 모듈로 보존해 다른 제어 버전과의 충돌을 방지.
- 사용자 제공 226점 roundabout_circulating.csv.
- 기존 launch는 수정하지 않고 integrated_no_obstacle_v61.launch를 추가했다.

## 구조
센서 수신은 공통으로 실행한다. 기본 전역 경로 → 고속도로 경로 선택 → adaptive curvature 제어 → 미션 명령 선택 → 기존 신호 제어 → /ctrl_cmd → UDP 브리지 순서다. 회전교차로 active 구간에서는 v6.1 드라이버의 한 JSON 프레임에 담긴 페달·조향을 선택한다. v6.1 드라이버는 직접 /ctrl_cmd에 발행하지 않는다. 미션 명령 선택기는 판단/명령이 오래되면 제동한다.

Frenet 회피, 터널 패치, 기존 간이 회전교차로 gate는 실행하지 않는다. 일반 구간에 새 장애물 우회 기능은 없고, 고속도로/회전교차로의 원래 차량 판단은 유지된다.

고속도로 진입 요청은 기존 route_mission_regions.json의 구간(누적거리 1118.742~1741.721m)을 사용한다. 첨부 고속도로 노드의 차선변경·차로 유지·복귀 동작을 유지한다. 경로가 변경되면 구간도 다시 검증해야 한다. 반복 주행 및 미션 중간 텔레포트는 미검증이다.

## 설치 (기존 코드를 덮어쓰지 말고 새 폴더)
압축의 AutoVehicle/morai_ws를 새 위치에 해제한다. 기존 ROS Noetic GPU 도커에서 사용하며 호스트 네트워크와 카메라 의존성이 필요하다. 같은 이름의 패키지가 있는 기존 작업공간을 동시에 source하지 않는다.

```bash
cd /새경로/AutoVehicle/morai_ws
source /opt/ros/noetic/setup.bash
python3 docker/final_ws/fetch_highway_model.py
catkin_make
bash RUN_INTEGRATED_NO_OBSTACLE.sh enable_control:=false
```

모델 파일은 용량 때문에 배포 ZIP에서 제외했다. 위 명령은 highway_best.pt만 준비한다. 기존 설치의 lane/lane_seg_best.pt와 models의 YOLO 가중치도 해당 위치에 복사해야 한다. 기존 Docker의 torch/OpenCV/ultralytics 등은 저장소 Dockerfile을 참고한다. enable_control=false는 최종 UDP 제어 송신을 끄고 내부 계산은 수행한다.

센서 입력과 /no_obstacle_integration/status, /roundabout_merge_gate/status, /highway_lane_strategy/state를 확인한 뒤 저속 시험:
```bash
bash RUN_INTEGRATED_NO_OBSTACLE.sh enable_control:=true max_speed_kph:=7.2
```
회전교차로 목표 속도는 별도 config/integration_v61.yaml의 v6.1 설정을 따른다. max_speed_kph는 일반/고속도로 제어에 적용된다. 저장소 v61_speedtest.yaml 값을 사용했으며 도커에서 별도 조정한 값을 추정 적용하지 않았다.

## 통신
MORAI Windows 192.168.10.14, Ubuntu 192.168.10.21, bind 0.0.0.0.
GPS 3001, IMU 4001, LiDAR 2001, 차선 카메라 1101, 신호 카메라 1131, Ego Vehicle Status 1911. 제어 목적지 9093 / 송신 바인드 9094.

## 검사 및 한계
integration_tests/test_no_obstacle.py: 명령 선택, 오래된 입력 제동, 회전교차로에서 일반 명령으로 잘못 대체하지 않음, 탈출 후 신선한 명령 필요, launch 필수 노드 검사.
Python 문법 및 4430점 전역 경로/226점 순환 경로의 v6.1 로딩 검사 수행.
실제 빌드, 메시지 생성, 카메라 모델 로딩, UDP 수신, 조향·가속 응답과 신호/미션 전환은 현장 검증 필요.

## 기록
별도 도커 터미널에서 ROS 환경 source 후:
```bash
rosbag record -O integrated_trial.bag /localization/odometry /Ego_topic /perception/lidar/tracked_obstacles_map /perception/camera/lane_info /highway_lane_strategy/state /roundabout_merge_gate/status /roundabout_test_driver/status /no_obstacle_integration/status /integration/nominal_cmd /control/ctrl_cmd /ctrl_cmd
```
