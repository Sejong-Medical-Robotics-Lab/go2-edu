#!/usr/bin/env python3
"""예제 — 최근접 장애물 거리 출력 (교재 6.5 도전 미션 '경로 2'의 그 예제).

/go2/pointcloud (PointCloud2) 를 구독해 전방 부채꼴(기본 ±45°) 안에서
가장 가까운 점까지의 거리를 주기적으로 출력한다. 바닥 점은 높이로 걸러낸다.

⚠ 이 예제는 '숫자를 읽는 것'까지만 한다 — 그 숫자에
"0.5 m 미만이면 StopMove" 규칙을 붙이는 것은 7장 도전 과제, 여러분의 몫이다.

    ros2 launch go2_gazebo sim.launch.py      # 터미널 1
    ros2 run go2_sim nearest_obstacle      # 터미널 2
"""
import argparse
import math
import struct
import sys


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--sector", type=float, default=45.0,
                    help="전방 부채꼴 반각 [deg]")
    ap.add_argument("--min-z", type=float, default=-0.25,
                    help="이보다 낮은 점(바닥)은 제외 [m, lidar 기준]")
    ap.add_argument("--rate", type=float, default=2.0, help="출력 주기 [Hz]")
    args, _ = ap.parse_known_args(argv)

    try:
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import PointCloud2
    except ImportError as e:
        sys.exit(f"ROS 2 환경이 필요합니다({e}) — source 와 colcon build 확인 "
                 "(교재 3.4).")

    half = math.radians(args.sector)

    class Watch(Node):
        def __init__(self):
            super().__init__("nearest_obstacle")
            self.create_subscription(PointCloud2, "/go2/pointcloud",
                                     self.cb, 5)
            self._last = self.get_clock().now()
            print(f"전방 ±{args.sector:.0f}° 최근접 거리 (바닥 z<{args.min_z} 제외)")

        def cb(self, msg: PointCloud2):
            now = self.get_clock().now()
            if (now - self._last).nanoseconds < 1e9 / args.rate:
                return
            self._last = now
            step = msg.point_step
            d_min, n = None, 0
            for i in range(msg.width):
                x, y, z = struct.unpack_from("<fff", msg.data, i * step)
                if z < args.min_z:                 # 바닥 제거 (6.4 독법 1단계)
                    continue
                if abs(math.atan2(y, x)) > half:   # 전방 부채꼴만
                    continue
                d = math.hypot(x, y)
                n += 1
                if d_min is None or d < d_min:
                    d_min = d               # ← 최소 거리를 계산하는 줄 (경로 2)
            if d_min is None:
                print(f"전방 {args.sector:.0f}° 안에 장애물 점 없음")
            else:
                print(f"최근접 장애물: {d_min:5.2f} m   (부채꼴 안 점 {n}개)")

    rclpy.init()
    try:
        rclpy.spin(Watch())
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
