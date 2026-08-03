# Go2 교육용 저장소 (go2-edu)

세종대 로봇 플랫폼 교육 프로그램 — **Go2 사족보행 로테이션 주간(교재 ③)** 실습
저장소입니다. **Gazebo(Classic)** 위에 Go2 모델·미션 월드가 올라가고, sport
mode 관제 노드가 교재의 토픽 조사·teleop·LiDAR 시각화·sport client API·미션
주행을 지원합니다.

> ⚠ **시뮬레이션 전용입니다.** 실기체 연결·조작은 금요일에 멘토의 검증된
> 스택으로만 합니다(교재 2.3). 그리고 교재 4.5 미션 ①: **이 README를 먼저
> 끝까지 통독한 뒤** 설치를 시작하세요.

⚠ **추가 공지 안내** 
기존 README의 설치 절차를 따를 때 일부 환경에서 오류가 발생할 수 있다는 보고가
있어, 별도의 [Gazebo 설치 가이드](./Gazebo%20설치%20가이드.md)를 따로 작성해 
두었습니다. Gazebo 설치 시 해당 가이드를 참고해주세요.

---

## 1. 교재 ↔ 저장소 대응표

| 교재 위치 | 할 일 | 저장소에서 쓰는 것 |
|---|---|---|
| 3.4 미션 A | 클론 → colcon build → 토픽 조사 5단계 | §3 설치, §6 토픽 표 |
| 4.2~4.5 | Gazebo 개념·설치·GUI(⏸/리셋/RTF)·월드 지도 | §4, Gazebo GUI 그대로 |
| 4.5 도전 | 월드 파일 열어 `<pose>` 확인 | `go2_gazebo/worlds/mission.world` |
| 4.4 체크포인트 ② | teleop 4방향 + 키를 떼면? | §5 teleop |
| 4.4 체크포인트 ④ | topic pub 으로 원 그리기 | §6 끝의 예시 명령 |
| 5.2~5.5 미션 B | sport client API · '3초 전진 후 정지' 해부 | `walk_demo`, §7 |
| 6장 | LiDAR → rviz2 (frame_id · 두 시점) | `/go2/pointcloud`, tf |
| 6.5 도전 경로 2 | 점구름 최소거리 계산 줄 찾기 | `nearest_obstacle` |
| 7장 표준 과제 | 시간 기반 시퀀스 순회 | `waypoint_template` |
| 7.5 인수인계 | HANDOVER.md 3항목 | `HANDOVER_TEMPLATE.md` |
| 부록 B | 빈출 트러블 | §10 트러블슈팅 |

## 2. 요구 환경

Ubuntu 22.04 + ROS 2 Humble(교재 ① 3장에서 준비한 환경). WSL2도 동작하나
Gazebo RTF가 낮을 수 있습니다 — 교재 4.3의 WSL2 안내를 따르세요. 

## 3. 설치 (교재 3.4 미션 ① / 4.3)

```bash
# 0) 클론 (조 공지의 저장소 주소)
cd ~/robot_ws/src
git clone <저장소 주소>          # → ~/robot_ws/src/go2-edu

# 1) 시뮬레이터 관련 의존성 (교재 4.3의 그 명령)
sudo apt update
sudo apt install -y ros-humble-gazebo-ros-pkgs ros-humble-teleop-twist-keyboard

# 2) 빌드 및 환경 적용
cd ~/robot_ws
colcon build --packages-select go2_edu_interfaces go2_description go2_gazebo go2_sim
source install/setup.bash        # 새 터미널마다! (또는 ~/.bashrc 에 추가)
```

설치 확인(교재 4.3 확인 기준 세 가지):

```bash
ros2 launch go2_gazebo sim.launch.py
# ① Gazebo 창에 월드(벽·박스·waypoint 마커)와 Go2가 보인다
# ② 터미널에 빨간 오류가 반복 출력되지 않는다
# ③ 다른 터미널에서:
ros2 topic list          # 3장에서 조사한 토픽들이 나타난다
```

첫 실행은 Gazebo 모델 캐시 준비로 수십 초 걸릴 수 있습니다. 로봇은
**엎드린 자세로 스폰**됩니다(스폰 직후 살짝 내려앉는 것은 정상).

## 4. 빠른 시작

```bash
ros2 launch go2_gazebo sim.launch.py             # Gazebo GUI + 로봇 + 관제 노드
ros2 launch go2_gazebo sim.launch.py gui:=false  # 헤드리스(WSL2 저성능 대안)
ros2 launch go2_gazebo sim.launch.py max_v:=0.5  # 멘토 속도 상한 (교재 5.1)
```

- **일시정지/재개·리셋·RTF**는 Gazebo GUI 그대로입니다(교재 4.2의 표):
  하단 ▶/⏸ 버튼, Edit→Reset World(Ctrl+R), 하단 Real Time Factor 표시.
- 시뮬 시간: gzserver가 `/clock`을 발행하고 모든 노드가 `use_sim_time`으로
  따릅니다 — 일시정지하면 상태 토픽의 stamp도 함께 멈춥니다(교재 4.2).
- **월드 리셋 후에는** 로봇 자세와 관제 상태를 맞추기 위해 API `Reset()`
  (또는 `ros2 topic pub -1 /go2/api go2_edu_interfaces/msg/ApiCommand
  "{name: Reset}"`)을 한 번 보내고 `StandUp` 부터 다시 하세요(§10).
- 일으키는 가장 빠른 방법:

```bash
ros2 run go2_sim walk_demo        # 기립→3초 전진→정지→엎드림 (교재 5.4 예제)
```

## 5. teleop — 키보드 조종 (체크포인트 ②)

```bash
# 시뮬레이터가 떠 있고, 로봇이 '기립 상태'일 때 (walk_demo 로 세우거나 §7):
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

- 키맵은 teleop 화면에 나옵니다(i/,=전·후진, j/l=회전, shift 조합=게걸음,
  k=정지). **교재 5.1의 키맵 확인 양식을 직접 채우는 것이 미션**입니다 —
  echo 창을 옆에 두고: `ros2 topic echo /cmd_vel`
- **키를 떼면?** — 직접 확인하세요(체크포인트 ②의 숨은 항목).
  힌트: 이 저장소의 `/cmd_vel` 경로는 **0.5초 워치독형**입니다(§7과 다른 점!).
- 엎드린 상태에서는 속도 명령이 **무시**되고 관제 노드가 경고를 남깁니다
  (교재 5.4 "기립 전 Move").

## 6. ROS 2 창구 — 토픽 표 (미션 A의 지도)

이름은 외우지 말고 `ros2 topic list -t` 로 직접 확인하는 것이 원칙(교재 3.1)
— 아래 표는 조사 결과를 채점할 때의 정답지입니다.

| 데이터 (교재 3.1) | 토픽 | 타입 | 주기 | 방향 |
|---|---|---|---|---|
| 속도 명령 | `/cmd_vel` | `geometry_msgs/Twist` | (내가 발행) | 내 코드 → Go2 |
| 보행 상태 | `/go2/sportmodestate` | `go2_edu_interfaces/SportModeState` | 20 Hz | Go2 → 내 코드 |
| LiDAR | `/go2/pointcloud` | `sensor_msgs/PointCloud2` (frame_id=`lidar_link`) | 10 Hz | Go2 → 내 코드 |
| 관절 상태(low) | `/go2/lowstate` | `go2_edu_interfaces/LowState` (읽기 전용!) | 50 Hz | Go2 → 내 코드 |
| 카메라 영상 | **없음** — 미션 ②에 "없음"이라 기록(시뮬↔실기체 차이 자체가 관찰) | — | — | — |

부가 토픽: `/odom`(오도메트리 **추정** — 드리프트 있음, 교재 3.2 ④),
`/go2/imu`, `/joint_states`, `/clock`, `/odom_gt`(Gazebo가 계산한 참값 —
멘토 시연용: `/odom` 과 겹쳐 보면 드리프트가 눈에 보입니다),
tf `odom → base → … → lidar_link`.

rviz2(체크포인트 ③): Fixed Frame 을 `lidar_link` ↔ `odom` 으로 바꿔 두 시점을
비교하는 것이 6.5 미션 ②입니다. Add→TF 를 켜면 좌표계 나무가 보입니다.
시뮬레이터를 재시작했다면 rviz2 도 함께 재시작(교재 4.2 sim time).

체크포인트 ④(원 그리기) — turtlesim 의 그 명령 그대로:

```bash
ros2 topic pub --rate 5 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.3}, angular: {z: 0.5}}"
# 기대 반경 = v/ω = 0.6 m. 멈출 때: Ctrl+C 후 정지 명령까지가 한 세트
```

## 7. sport client API (교재 5.2)

```python
from go2_sim.client import SportClient
client = SportClient()        # ① 객체를 만들고 (시뮬레이터가 떠 있어야 함)
client.StandUp()              # ② 함수를 부른다
client.Move(0.3, 0, 0)        #    vx, vy, vyaw — 몸통 기준 (교재 5.3)
client.StopMove()
client.StandDown()
```

지원 함수: `StandUp / StandDown / Move / StopMove / BalanceStand /
GetMode / GetState / WaitMode / Reset(시뮬 전용)`. 함수 호출은 내부적으로
`/go2/api` 토픽으로 번역됩니다 — `ros2 topic echo /go2/api` 를 켜 두고 API 를
부르면 "함수 뒤에도 메시지가 흐른다"(교재 3.1)를 눈으로 확인할 수 있습니다.

**명령 유지 방식(미션 B ①의 조사 대상)** — 두 경로가 다릅니다:
`Move()` 는 **1회 유지형**(StopMove 까지 계속 걷는다 — 예제 ⑥번 줄이 안전의
핵심), `/cmd_vel` 은 **0.5초 워치독형**(계속 보내야 유지). 왜 이렇게 나눴을지
조에서 토론해 보세요.

## 8. 관찰 노트

- 속도 명령은 내장 제어기(planar 구동)가 그대로 실행하므로 예측 산수가 잘
  맞습니다: vyaw=0.5 로 90° ≈ 3.1 s — 스톱워치로 검증해 보세요(교재 5.3).
  약간의 가감속 램프(안전) 때문에 아주 짧은 구간에서는 예측보다 조금 덜
  갑니다.
- `/odom`(추정)은 오래 걸을수록 실제(`/odom_gt`)와 벌어집니다 — 사각형 왕복
  후 두 값을 비교해 보세요(5.5 도전·7.3 단계 3).
- 다리는 관제 노드가 trot 패턴으로 구동합니다 — 대각선 짝(FL+RR / FR+RL)이
  정말 함께 움직이는지 눈으로 확인해 보세요(교재 2.2). 지지발이 살짝
  미끄러지듯 보이는 것은 이 교육용 스택의 단순화입니다(멘토 문서 참고).
- 장애물·벽에 부딪히면 로봇이 막히거나 밀립니다 — "점구름에 없다고 장애물이
  없는 것은 아니다"(교재 6.1). 자세가 이상해졌으면 §10의 리셋 절차.

## 9. 저장소 구조

```
go2-edu/
├── go2_description/        # Unitree URDF+메시 (BSD-3, 원본 무수정)
├── go2_gazebo/             # 월드(mission.world)·시뮬 URDF·launch
│   └── urdf/go2_sim.urdf   #   자동 생성본 (tools/gen_sim_urdf.py)
├── go2_sim/                # sport mode 관제 노드·SportClient·예제 3종
├── go2_edu_interfaces/     # msg 5종 (colcon)
├── docs/                   # joint_map · real_robot(멘토) · textbook_notes
└── HANDOVER_TEMPLATE.md    # 인수인계 3항목 (교재 7.5)
```

## 10. 트러블슈팅 (부록 B 대응)

| 증상 | 확인/해결 |
|---|---|
| launch 파일을 못 찾음 | colcon build 후 **모든 터미널**에서 `source ~/robot_ws/install/setup.bash` (교재 4.3 표 1행) |
| Gazebo 창이 검게/느리게 뜸 | RTF 확인 · `nvidia-smi` · WSL2면 `gui:=false` + rviz2 관찰 (교재 4.3) |
| 로봇이 넘어진/이상한 자세 | Gazebo Edit→Reset World 후 API `Reset()` → `StandUp` (부록 B ①) |
| topic pub 을 보냈는데 안 움직임 | ① 토픽 이름·타입 ② **기립 상태인가**(GetMode/상태 토픽) ③ teleop 이 0 속도를 계속 덮어쓰고 있지 않은가 (부록 B ②) |
| rviz2 에 점구름이 안 보임 | 교재 6.3 점검 4순서: `ros2 topic hz /go2/pointcloud` → Fixed Frame(`lidar_link`) → Display 추가 → sim time(시뮬 재시작 시 rviz2 도 재시작) |
| topic list 가 비어 있음 | 시뮬레이터가 완전히 뜬 뒤인지 · `ROS_DOMAIN_ID` 가 터미널마다 같은지 (교재 2.4) |
| spawn 실패/로봇이 안 보임 | 이전 gzserver 잔존 여부: `pkill -f gzserver` 후 재실행 |
| SportClient "시뮬레이터가 보이지 않습니다" | 터미널 1에서 sim.launch.py 를 먼저 |

## 11. 라이선스

코드 MIT(`LICENSE`) · Go2 모델(`go2_description/`)은 Unitree BSD-3-Clause —
`THIRD_PARTY_NOTICES.md` 참고.
