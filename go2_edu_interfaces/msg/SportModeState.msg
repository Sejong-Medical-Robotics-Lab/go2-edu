# 보행 상태 보고 (교재 3.2 '상태 토픽 해부'의 대상)
# position은 오도메트리 추정치 — 드리프트가 있다 (교재 3.2 ④)
std_msgs/Header header            # frame_id = odom
string mode                       # lying | standing_up | standing | moving | lying_down | fallen
string error                      # 예: FALL_DETECTED — ... (없으면 빈 문자열)
geometry_msgs/Vector3 velocity    # 몸통 기준 실측 속도: x=vx, y=vy, z=vyaw
geometry_msgs/Vector3 position    # odom 기준 추정 위치 (z는 0)
float32 yaw                       # odom 기준 추정 요 [rad]
