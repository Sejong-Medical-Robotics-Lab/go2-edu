# 실기체(Go2)와의 관계 · 스택 내부 — 멘토·조교용

> 학생 배포용 아님. 학생에게는 "실기체는 금요일, 멘토의 검증된 스택으로만"
> (교재 2.3)까지만 전달합니다.

## 개념 매핑 (시뮬 ↔ 실기체)

| 이 저장소 | 실기체 스택 | 비고 |
|---|---|---|
| `SportClient.StandUp/Move/…` | unitree_sdk2(_python) go2 sport client | 함수 '모양'을 맞춘 교육용. 시그니처·지원 동작은 펌웨어/SDK 버전 확인 필수 |
| `/go2/sportmodestate` | DDS `rt/sportmodestate` | 축약 메시지(모드·속도·위치·요) |
| `/go2/lowstate` | DDS `rt/lowstate` | 12모터 순서 동일(docs/joint_map.md) — Gazebo `/joint_states`를 이름으로 재정렬 |
| `/cmd_vel` 워치독 0.5s | 실기체의 안전 타임아웃 관행 | 값은 다를 수 있음 |
| `/go2/pointcloud` | L1 드라이버 토픽 | 실기체는 QoS best-effort 계열일 수 있음(교재 3.3 '복병'은 금요일에 만남) |
| `Reset()` / Gazebo Reset World | 없음(물리 세계!) | 시뮬 전용임을 학생에게 상기 |

## 스택 아키텍처와 설계 근거

물리 보행 제어기를 Gazebo에서 새로 튜닝하는 대신, **베이스는
`planar_move` 플러그인이 명령 속도 그대로(기구학) 구동**하고, **다리는
`joint_pose_trajectory` 플러그인이 관제 노드의 trot 목표를 기구학적으로
설정**합니다. 접촉은 base(장애물 차단)와 4개 발(지면 지지·저마찰)만 남겼습니다.

- 장점: PID/접촉 튜닝 리스크 0, 속도 명령↔이동의 정합이 구조적으로 보장
  (교재 7.3 단계 1의 ±20% 기준, 5.3 회전 산수) — 수요일이 안전해짐.
- 대가: 지지발이 미끄러지듯 보일 수 있음(다리는 시각·관찰용 애니메이션),
  로봇이 '넘어지는' 물리는 없음(부록 B ①은 Reset 절차로 재해석 — README §10).
- 교육 목표 정합: 교재 2.2 "이번 주에 보행 알고리즘을 구현하지는 않는다",
  이번 주의 계층은 high-level(sport mode)뿐 — 보행 물리는 학습 범위 밖.

trot 다리 패턴·IK는 MuJoCo 동역학 시뮬로 선행 검증한 것을 이식했습니다
(IK-FK 대조 0.00 mm, 속도장 -(v+ω×r), 접지 무긁힘 프로파일).

## 오프라인 검증 완료 (이 저장소를 만든 환경에서 실측)

- `go2_sim.urdf`: XML·urdf_parser_py 파스, 링크-조인트 트리 정합, revolute
  12개(=DDS 순서), collision = base+4feet 만, 플러그인 5종·리매핑 존재
- `mission.world`: XML 파스 (장애물 3·벽 4·waypoint 3, `<pose>` 도전 성립)
- 기구학 이식: 선행 검증본과 IK·stand_pose 수치차 0
- gait 스트림 6초: 관절 한계 내, 스텝당 최대 점프 0.081 rad(연속)
- SupervisorCore 단위검증 8항목: 기립 전 Move 거부 / StandUp 1.2 s /
  API 유지형 / StopMove 복귀 / cmd_vel 워치독(재전송 유지·단절 0.5 s 정지) /
  사각형 왕복 드리프트(오도 0.063 m vs 참 0.026 m) / Reset / 90° 회전 89.4°
- 전 파이썬 파일 컴파일, launch 파일 컴파일

## 실환경 미검증 — 최초 1회 점검 절차 (멘토, 배포 전 필수)

이 저장소는 ROS/Gazebo가 없는 환경에서 작성되어 **실행 검증이 되어 있지
않습니다.** Humble 표준 패턴만 사용했지만, 아래를 한 번 통과시킨 뒤
배포하세요(30분 예상).

1. `colcon build` 4개 패키지 → `source install/setup.bash`
2. `ros2 launch go2_gazebo sim.launch.py` — 월드·로봇 로드, 터미널 오류 확인.
   플러그인 파라미터 미인식 **경고**가 떠도 기능이 동작하면 무시 가능
   (배포판별 파라미터 명칭 차이 — 아래 '조정 노브')
3. `ros2 topic list` 에서: /cmd_vel /go2/sportmodestate /go2/lowstate
   /go2/pointcloud /go2/api /odom /odom_gt /joint_states /go2/imu /clock
4. `ros2 run go2_sim walk_demo` — 기립→전진→정지→엎드림 + 오도메트리 출력
5. teleop 전·후·회전·게걸음, 키 떼고 0.5 s 정지 확인
6. rviz2: Fixed Frame `lidar_link` 로 점구름, `odom` 으로 두 시점 비교
7. `ros2 topic pub --rate 5 /cmd_vel …` 원 그리기(반경 ≈0.6 m)
8. Gazebo Reset World → `Reset()` → `StandUp` 절차 확인

## 조정 노브 (현장 이슈 대응)

- 지지발 스케이팅이 거슬림 → `tools/gen_sim_urdf.py` 의 발 `mu1/mu2`(0.05)
  하향 또는 `kp` 조정 후 재생성
- trot 중 몸통 미세 흔들림 → 같은 파일 base `dampingFactor`(0.01) 상향
- planar_move 가 `/odom_gt` 를 안 냄 → 해당 배포판의 파라미터 로그 확인;
  odom 리매핑만 유지해도 교육 기능(우리 `/odom`)은 무영향
- LiDAR 부하 → `LIDAR_H_SAMPLES`(120)·`LIDAR_V_SAMPLES`(8) 축소 후 재생성
- 다리 목표 미반영 → `ros2 topic info /go2/set_joint_trajectory` 로 플러그인
  구독 여부 확인(리매핑 오탈자 점검)

## 실기체 세션 전 점검

- [ ] SDK·펌웨어 버전 확인, 검증된 데모만 사용 · 리모컨 비상정지 소지자 지정
- [ ] 네트워크(로봇 IP·ping)·ROS_DOMAIN_ID 분리(교재 2.4 3단계 점검)
- [ ] 시뮬↔실기체 차이 브리핑: 실제 보행 동역학·마찰·지연·배터리,
      LiDAR 노이즈·QoS, 그리고 "Reset은 현실에 없다"
