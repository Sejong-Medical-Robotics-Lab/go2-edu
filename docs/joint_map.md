# Go2 관절 인덱스 맵 (12 DOF)

- 이 표의 **인덱스 = /go2/lowstate 의 `motor_state` 배열 순서 = 실기체 DDS 12자유도 순서**입니다.
- Gazebo `/joint_states` 는 순서를 보장하지 않으므로, 관제 노드가 이름으로 재정렬해 발행합니다(`go2_sim/supervisor.py`).
- 다리: FR(앞오른쪽)·FL(앞왼쪽)·RR(뒤오른쪽)·RL(뒤왼쪽), 다리당 (hip=고관절 좌우, thigh=고관절 앞뒤, calf=무릎) — 교재 2.1

| idx | 관절 이름 (URDF/DDS 동일) | 다리 |
|---:|---|---|
| 0 | `FR_hip_joint` | FR |
| 1 | `FR_thigh_joint` | FR |
| 2 | `FR_calf_joint` | FR |
| 3 | `FL_hip_joint` | FL |
| 4 | `FL_thigh_joint` | FL |
| 5 | `FL_calf_joint` | FL |
| 6 | `RR_hip_joint` | RR |
| 7 | `RR_thigh_joint` | RR |
| 8 | `RR_calf_joint` | RR |
| 9 | `RL_hip_joint` | RL |
| 10 | `RL_thigh_joint` | RL |
| 11 | `RL_calf_joint` | RL |
