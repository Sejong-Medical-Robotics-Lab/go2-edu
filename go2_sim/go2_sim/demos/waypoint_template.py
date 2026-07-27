#!/usr/bin/env python3
"""예제 — 시퀀스 실행 템플릿 (교재 7.2 '시퀀스 표'를 코드로).

설계 시트의 표를 SEQUENCE 리스트에 한 줄씩 옮긴다.
행 형식: ("move", (vx, vy, vyaw), 시간[s]) / ("stop"|"standup"|"standdown", None, 대기[s])

지금은 예시로 '1구간 전진'만 채워져 있다 — 나머지는 여러분의 설계다
(교재 7.2 설계 회의 → 7.3 단계 3). 회전 산수: 90° = 1.571 rad,
vyaw=0.5 면 약 3.14 s (교재 5.3).

    ros2 launch go2_gazebo sim.launch.py    # 터미널 1
    ros2 run go2_sim waypoint_template      # 터미널 2
"""
import argparse
import time

from go2_sim.client import SportClient

# ── 여기를 채우세요 (교재 7.2 시퀀스 표) ─────────────────────────
SEQUENCE = [
    ("standup", None,            2.5),
    ("move",    (0.3, 0.0, 0.0), 3.33),   # 예시: 약 1 m 전진
    ("stop",    None,            1.0),
    # TODO: WP1 → WP2 → WP3 → 복귀 구간을 설계 시트대로 추가
    ("standdown", None,          2.0),
]
# ────────────────────────────────────────────────────────────────


def main(argv=None):
    argparse.ArgumentParser(description=__doc__).parse_known_args(argv)
    c = SportClient()
    st0 = c.GetState()
    for i, (kind, arg, wait) in enumerate(SEQUENCE, 1):
        print(f"[{i}/{len(SEQUENCE)}] {kind} {arg or ''} (모드: {c.GetMode()})")
        if kind == "standup":
            c.StandUp()
        elif kind == "standdown":
            c.StandDown()
        elif kind == "move":
            c.Move(*arg)
        elif kind == "stop":
            c.StopMove()
        time.sleep(wait)
    c.StopMove()
    st1 = c.GetState()
    if st0 and st1:
        dx = st1[2][0] - st0[2][0]
        dy = st1[2][1] - st0[2][1]
        print(f"오도메트리 기준 출발점 대비: ({dx:+.2f}, {dy:+.2f}) m "
              "— 복귀 시퀀스라면 이 값이 곧 왕복 오차 (7.3 단계 3)")


if __name__ == "__main__":
    main()
