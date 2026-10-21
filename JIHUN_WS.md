# jihun_ws — 현재 사용하는 주행 기능 모음

`jihun_ws`는 2026-10-21 정지선 접근 감속 수정이 포함된 `final_ws`에서 분기했다.
사용하는 구성은 **2번 `curvature_signal`**이다. 다른 기능과 병합할 때 공통
코드의 이력을 유지할 수 있도록 기존 패키지 경로를 유지한다. 주행 코드를
복사해 별도 구현하지 않고, 아래 실행 파일에서 현재 사용하는 구성만 선택한다.

## 실행

```bash
cd ~/AutoVehicle
git fetch origin
git switch --track origin/jihun_ws   # 최초 전환 시
cd morai_ws/docker/jihun_ws
bash run_jihun.sh approach smooth    # 저장된 감속 값과 정차 여유 0.5 m 적용
bash run_jihun.sh stop
bash run_jihun.sh install --config
bash run_jihun.sh start
bash run_jihun.sh drive
```

이미 로컬 브랜치가 있으면 `git switch jihun_ws` 후
`git pull --ff-only origin jihun_ws`로 갱신한다. 전환 전 주행 launch는 `Ctrl+C`로 종료한다.
모든 경로는 기존 `morai_ws/docker/final_ws/highway.env`와 `highway-test.env`를
공유한다. 기존 `bash run_test.sh drive 2`도 같은 코드를 실행한다.

| 사용 동작 | 명령 |
| --- | --- |
| 설정 확인, 제어 명령 전송 없음 | `bash run_jihun.sh show` |
| 카메라·경로·제어 계산 모니터 | `bash run_jihun.sh monitor` |
| 현재 구성으로 자율주행 | `bash run_jihun.sh drive` |
| 통신·속도·주행 제약 진단 | `bash run_jihun.sh diagnose` |
| 최고속도 설정 | `bash run_jihun.sh speed 35` |
| 거리 기반 정지 접근 설정 | `bash run_jihun.sh approach smooth` |
| 앞범퍼 정차 여유 설정 | `bash run_jihun.sh clearance 0.5` |
| CAM1 경로 잔차 조향 보정 | `bash run_jihun.sh lane on` 또는 `off` |
| 기존 커브 주행 설정 | `bash run_jihun.sh turn mission` 또는 `safe` |
| 웹 카메라·정지 원인 보기 | `bash run_jihun.sh view` |
| 모델 점검 | `bash run_jihun.sh models` |

`turn mission`의 최고속도는 55 km/h이며, 실제 목표속도는 곡률과 교차로 조건에
따라 내려간다. `approach smooth`는 최고속도를 변경하지 않는다. `safe`는 더 이른
감속을 위한 보수적 설정이므로 사용 후 빠른 정지 접근을 원하면 `approach smooth`를 적용한다.

## 포함하는 기능과 병합 시 확인할 위치

| 기능 | 주 구현 위치 |
| --- | --- |
| 고정 MGeo 경로 추종, 끝→처음 회차, 곡률 속도 조절 | `morai_ws/src/experimental/curvature_speed_purepursuit/` |
| Ego 차량 실제 속도와 가감속 페달 제어 | `morai_ws/src/control/purepursuit_mgeo/src/purepursuit_mgeo/longitudinal_controller.py` |
| CAM1 차선·정지선, 선택적 잔차 조향 보정 | `morai_ws/src/detection/camera_perception/`, `curvature_speed_purepursuit/lane_centering.py` |
| CAM4 개선 모델, 지도상 현재 진행 방향과 신호 대응 | `camera_perception/`, `morai_ws/src/control/turn_signal_controller/` |
| 3번째 교차로의 같은 위상 신호등 처리 | `morai_ws/config/curvature_signal.yaml`의 `correlated_signal_heads`, `signal_association.py` |
| 4→5번째 교차로 분리, 반대 차선 정지선 배제 | `maneuver_fusion_node.py`, `route_context.py`, `signal_association.py` |
| 통과 신호 확인·진입 확정·진입 후 신호 재정차 방지 | `morai_ws/src/control/turn_signal_controller/scripts/maneuver_fusion_node.py` |
| 점진적 정지선 접근, 앞범퍼 0.5 m 여유 | `morai_ws/src/control/stopline_control/src/stopline_control/core.py` |
| GPS/IMU/차량 상태 수신, MGeo 좌표, 위치 추정 | `morai_ws/src/localization/{morai_udp_bridge,gps_mgeo_converter,ekf_local_enu}/` |
| 최종 제어 전송과 AV 모드 요청 | `morai_ws/src/localization/morai_udp_drive_bridge/` |
| 카메라·실제 속도·목표속도·정지 원인 웹 표시 | `morai_ws/src/detection/camera_perception/web/`, `debug_dashboard.py` |
| 설치·파일 해시 일치 확인·진단 | `morai_ws/docker/final_ws/` |

경로와 교차로 기준 파일은 `morai_ws/data/routes/2026_molit_comp_global_path.txt`와
`morai_ws/config/curvature_signal.yaml`이다. 위 표의 축약 경로는 해당 패키지 내부를
뜻한다. 기계가 읽을 수 있는 목록은 `morai_ws/docker/jihun_ws/features.json`에 있다.
설치에 필요한 전체 파일 목록은 `curvature_runtime_files.sh`가 생성하므로 새 helper
파일도 설치 시 함께 포함된다.

이 실행 구성에서는 LiDAR 장애물 회피·자동 차선 변경·합류 요청·방향지시등 UDP를
활성화하지 않는다. 해당 공통 패키지는 다른 구성과의 병합 및 의존 관계를 위해
저장소에 유지한다. 현재 사용하는 기능 추가는 이 브랜치에서 진행하면 된다.

## PC별 설정

현재 알려진 주소는 Ubuntu `192.168.10.21`, MORAI Windows `192.168.10.14`이다.
두 주소와 실제 컨테이너 이름은 `morai_ws/docker/final_ws/highway.env`에서 설정한다.
이 파일과 `highway-test.env`는 Git에서 제외되므로 PC별 설정이 병합에 섞이지 않는다.
최고속도와 선택적 CAM1 보정의 현재 저장값은 `show`로 확인한다.

## 다른 기능과 병합

`jihun_ws`는 수정된 `final_ws`의 자손이므로 정지 제어 수정도 포함한다.
나중에 통합 브랜치에서 다음과 같이 병합한다.

```bash
git fetch origin
git merge origin/jihun_ws
```

같은 기능을 파일 복사로 덮어쓰지 말고 Git 병합으로 공통 변경을 확인한다.
특히 최종 `/ctrl_cmd` 발행기는 하나만 두고, 병합 후에는 launch의 제어 연결,
신호 진입 상태, 속도 단위, 런타임 해시를 확인한다. 브랜치 간 전환 후 컨테이너도
위 설치 절차로 갱신해야 해당 브랜치의 코드를 실행한다.

감속 제어와 확인 범위는 `morai_ws/docs/STOPLINE_APPROACH_20261021_KO.md`에 기록했다.
