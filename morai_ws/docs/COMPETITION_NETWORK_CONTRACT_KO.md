# 대회 v1.2 네트워크 계약

`merged_highway.launch`는 차량의 Network Settings에서 다음 세 항목만 사용한다.

| 항목 | MORAI Host Port | 참가팀 Destination/Source Port | 방향 |
|---|---:|---:|---|
| Ego Ctrl Cmd | 9093 | 9094 | Sim <- User |
| Collision Data | 9091 | 9092 | Sim -> User |
| Competition Vehicle Status | 9088 | 9099 | Sim -> User |

수신기는 MORAI PC의 IP와 Host Port를 모두 검사한다. Ctrl Cmd는 local UDP
9094에서 MORAI PC 9093으로 보내며 `longlCmdType=1`, `ctrl_mode=2`만 사용한다.

GPS, IMU, Camera, 3D LiDAR는 차량 Network Settings의 추가 항목이 아니라 규정
2-8의 4~7번에서 별도로 허용된 센서 UDP이므로 유지한다. 방향지시등, Object
Info, Traffic Light ground truth, Ego Vehicle Status 네트워크는 이 launch에서
실행하지 않는다.

Competition Vehicle Status는 위치, Y/Z 속도와 선형가속도 필드가 없는 별도
149바이트 계약으로 파싱한다. 타이어 필드까지 포함한 229바이트 Ego Vehicle
Status 패킷과 구형 코드의 181바이트 축약형은 모두 명시적으로 거부한다. 내부
ROS 메시지도 위치나 금지된 필드를 정의하지 않으며,
종방향 PI 제어의 측정 속도는 `/competition/vehicle_speed_mps`의 `vx`만 사용한다.

정적 코드 테스트는 포트 전파, legacy 패킷 거부, 송신자 포트 검사와 상태
timeout 시 full brake를 검증한다. 실제 대회 빌드 `25.S4.MolitComp03`의 정확한
헤더·필드 순서·빈 CollisionData 전송 방식은 MORAI가 실행되는 PC에서 packet
capture로 최종 확인해야 한다. 캡처 확인 전에는 코드의
`protocol_verified`를 `false`로 유지한다.
