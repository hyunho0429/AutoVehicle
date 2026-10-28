# 정지선까지 점진적으로 접근하기

적용 대상은 `bash run_test.sh drive 2`의 `curvature_signal` 구성이다.
정차 목표는 **앞범퍼에서 정지선까지 0.5 m**이다. 지도/차량 기준점과 카메라
거리의 오차가 있으므로 실제 MORAI 정차 위치는 별도로 확인해야 한다.

## 변경한 제어

- 정지선을 검출했다는 이유만으로 브레이크를 걸지 않는다.
- 남은 거리로부터 목표속도를 계산하고, 현재 속도가 그 속도를 넘을 때
  속도 오차에 비례해 제동을 시작한다. 멀리서 작은 오차가 생기면 제동도 작다.
- 목표속도보다 이미 느린 경우에는 제동을 풀어 정상 주행 제어기의 가속을
  허용한다. 먼 거리부터 1~2 km/h로 남은 거리를 이동하는 상황을 방지한다.
- 최대 제동으로도 남은 정지거리가 부족한 상황에는 즉시 최대 제동을 요청한다.
- 현재 경로에 맞는 통과 신호가 확정되면 기존처럼 정차 제한을 해제한다.
  교차로에 진입한 뒤 같은 교차로 신호를 다시 판단하지 않는 동작도 유지한다.
- 진입 후 재정차 방지는 보정된 카메라 구성과 기존 교차로 구성에 공통으로
  적용한다. 위치·장애물·차량 상태·상위 제어 명령의 이상은 계속 정지 사유가 된다.

목표속도는 `sqrt((a*tau)^2 + 2*a*d) - a*tau`이다.
`d`는 앞범퍼부터 정지선까지 거리에서 정차 여유 0.5 m를 뺀 값이며,
`a`는 계획 감속도, `tau`는 반응 여유 시간이다. 곡률/회전/최고속도 제한이
더 낮으면 그 제한이 우선한다. 정지선 근처에서 강제로 최저속도를 보장하지 않는다.

## 설정

| 저장값 | 새 기본값 | 의미 |
| --- | --- | --- |
| `STOPLINE_HOLD_DISTANCE_M` | `0.5` | 앞범퍼 기준 정차 여유(m) |
| `STOPLINE_PLANNING_DECEL_MPS2` | `1.2` | 정지선 속도 곡선의 감속도(m/s²) |
| `STOPLINE_BRAKE_RAMP_DISTANCE_M` | `0.0` | 추가 접근 준비 거리(m), 검출만으로 제동하지 않음 |
| `STOPLINE_SETTLE_DISTANCE_M` | `0.15` | 거의 정지한 차량의 최종 미세 이동 방지 범위(m) |

기존 `highway-test.env`는 업데이트해도 보존되므로 **다음 명령을 한 번 실행한다.**

```bash
cd ~/AutoVehicle/morai_ws/docker/final_ws
bash run_test.sh approach smooth
```

이 명령은 위 네 값을 저장하고 기존 설정을 `highway-test.env.bak`에 백업한다.
계획 감속도는 저장된 `MAX_DECEL_MPS2`보다 커지지 않는다.
최고속도·조향·카메라·IP 설정은 보존된다. 실행 중인 launch는 종료하고 다시 시작한다.
`turn mission`을 나중에 실행해도 정차 여유가 과거의 6 m로 바뀌지 않는다.

## 업데이트와 실행

```bash
cd ~/AutoVehicle
git pull --ff-only origin final_ws
cd morai_ws/docker/final_ws
bash run_test.sh approach smooth
bash run_highway.sh stop
bash install_curvature_signal.sh --config
bash run_highway.sh start
bash run_test.sh drive 2
```

기존 주행 터미널은 `Ctrl+C`로 종료한 상태에서 적용한다.
설치기는 정지된 컨테이너에 전체 의존 파일을 설치하고 해시를 확인한다.

## 확인 범위

`test_progressive_approach.py`는 이상적인 페달 모델에서 35/55 km/h 접근,
2 km/h에서의 접근 회복, 제동 효과가 더 큰 경우, 늦게 관측한 빨간불을 확인한다.
속도·카메라가 오래되었을 때 정지하고 새 초록불 확인 전에는 정차를 풀지 않는
기존 회귀 시나리오도 함께 실행한다. 실제 MORAI의 제동 반응을 측정한 결과는 아니다.
`test_progressive_approach_recipes.py`는 저장된 과거 설정이 실제 launch 인자로
바뀌는지 확인하며 차량 제어를 시작하지 않는다.

2026-10-21 확인: 정지 제어 59개, 교차로/신호 통합 210개, 저장 설정 전달 4개가
통과했다. 35 km/h로 시작한 이상적인 페달 모델에서는 20 m 전 약 25 km/h,
10 m 전 약 18 km/h, 5 m 전 약 12 km/h를 거쳐 앞범퍼 약 0.49 m 여유로 정지했다.
이 수치는 제어 순서를 확인하기 위한 모델 결과이며 MORAI 실측 속도가 아니다.
