# 최종 병합 오프라인 검증

확인일: 2026-10-29. 총 374개 테스트 통과.

- integration_tests/patches: 47개 통과
- integration_tests: 26개 통과
- src/experimental/curvature_speed_purepursuit/test: 32개 통과
- src/control/stopline_control/test: 59개 통과
- src/control/turn_signal_controller/test: 210개 통과

전체 src Python 구문 검사와 launch XML 파싱 통과. 실제 지도·경로·신호 설정과
동적 장애물 설정을 함께 읽어 최종 제어기 초기화 성공을 확인했다.
V9 heartbeat, 패치 연결, 7m 정차 목표, 좁은 차선의 실제 조향 보정,
의도적 차선 변경 시 보정 해제, 회차 종점 비정지, 속도 상승의 물리 상한을 검사했다.

ROS/catkin 빌드, 센서 스트리밍, 모델 추론 및 실제 MORAI 주행은 수행하지 않았다.
이 결과는 실제 정차 오차나 차선 비침범·대회 완주를 보증하는 주행 결과가 아니다.
