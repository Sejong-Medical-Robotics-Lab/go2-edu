#!/usr/bin/env python3
"""예제 — '3초 전진 후 정지' (교재 5.4 뼈대 · 5.5 미션 B의 그 스크립트).

    # 터미널 1: 시뮬레이터
    ros2 launch go2_gazebo sim.launch.py
    # 터미널 2:
    ros2 run go2_sim walk_demo
"""
import argparse
import time

from go2_sim.client import SportClient


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--vx", type=float, default=0.3)
    ap.add_argument("--dur", type=float, default=3.0)
    args, _ = ap.parse_known_args(argv)

    # ① 초기화 — 로봇과 대화할 채비
    client = SportClient()
    print("모드:", client.GetMode())

    # ② 기립 — 엎드린 로봇에게 Move 는 무의미
    client.StandUp()
    # ③ 잠시 대기 — 동작에는 시간이 걸린다
    time.sleep(2.0)
    print("기립 후 모드:", client.GetMode())

    # ④ 전진 시작 (몸통 기준 vx [m/s] — 교재 5.3 좌표계)
    st0 = client.GetState()
    client.Move(args.vx, 0.0, 0.0)
    # ⑤ 이동 유지 — 이 저장소의 Move 는 '유지형': sleep 만으로 계속 걷는다
    time.sleep(args.dur)
    # ⑥ 명시적 정지 — 이 줄이 안전의 핵심
    client.StopMove()
    time.sleep(1.0)

    st1 = client.GetState()
    if st0 and st1:
        dx = st1[2][0] - st0[2][0]
        dy = st1[2][1] - st0[2][1]
        d = (dx * dx + dy * dy) ** 0.5
        print(f"오도메트리 이동량: {d:.2f} m "
              f"(예측 {args.vx * args.dur:.2f} m 와 비교 — 미션 B ②)")
    # ⑦ (선택) 엎드려 마무리
    client.StandDown()
    time.sleep(2.0)
    print("종료 모드:", client.GetMode())


if __name__ == "__main__":
    main()
