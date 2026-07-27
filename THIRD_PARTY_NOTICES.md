# 제3자 라이선스 고지

## Unitree Go2 URDF·메시 (`go2_description/`)

- 출처: [unitreerobotics/unitree_ros](https://github.com/unitreerobotics/unitree_ros)
  의 `robots/go2_description` (urdf + dae)
- 라이선스: **BSD-3-Clause** — 원문 `go2_description/LICENSE.unitree_ros`
- 원본 `go2_description.urdf` 는 무수정 보존하며, 시뮬용 변형본
  `go2_gazebo/urdf/go2_sim.urdf` 는 `tools/gen_sim_urdf.py` 로 자동 생성됩니다
  (충돌 정리·lidar_link·Gazebo 플러그인 블록 추가 — 파일 머리말에 명시).

## API 명칭

`SportClient` 의 함수 이름(StandUp, Move 등)은 실기체 SDK(unitree_sdk2 /
unitree_sdk2_python)의 sport client 와 같은 '모양'이 되도록 지은 것이며,
해당 SDK 코드는 포함하지 않습니다.
