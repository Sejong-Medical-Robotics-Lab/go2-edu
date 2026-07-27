"""go2_sim.client — sport client API (교재 5.2의 그 창구).

    client = SportClient()          # ① 클라이언트 객체를 만들고
    client.StandUp()                # ② 함수를 부른다
    client.Move(0.3, 0, 0)          #    vx, vy, vyaw — 몸통 기준 (교재 5.3)
    client.StopMove()

함수 호출은 /go2/api 발행으로, 상태 조회는 /go2/sportmodestate 구독으로
번역된다 — "함수 뒤에도 결국 메시지가 흐른다"(교재 3.1).

Move 는 유지형이다: StopMove 까지 계속 걷는다 (교재 5.3 '명령의 유통기한'의
답 — API 경로는 1회 유지형, /cmd_vel 경로는 0.5초 워치독형).
"""
from __future__ import annotations

import time


class SportClient:
    def __init__(self):
        try:
            import rclpy
            from go2_edu_interfaces.msg import ApiCommand, SportModeState
        except ImportError as e:
            raise RuntimeError(
                f"ROS 2 환경이 필요합니다({e}). colcon build 와 "
                "`source install/setup.bash` 를 확인하세요 (교재 3.4).") from e
        if not rclpy.ok():
            rclpy.init(args=None)
        self._rclpy = rclpy
        self._node = rclpy.create_node("sport_client")
        self._pub = self._node.create_publisher(ApiCommand, "/go2/api", 10)
        self._state = None
        self._node.create_subscription(
            SportModeState, "/go2/sportmodestate", self._on_state, 10)
        self._Api = ApiCommand
        t0 = time.time()
        while self._pub.get_subscription_count() == 0 and time.time() - t0 < 3:
            rclpy.spin_once(self._node, timeout_sec=0.05)
        if self._pub.get_subscription_count() == 0:
            print("[SportClient] 경고: 시뮬레이터(go2_supervisor)가 보이지 "
                  "않습니다 — 터미널 1에서 sim.launch.py 를 먼저 실행하세요.")

    def _on_state(self, msg):
        self._state = msg

    def _send(self, name, vx=0.0, vy=0.0, vyaw=0.0):
        m = self._Api()
        m.name, m.vx, m.vy, m.vyaw = name, float(vx), float(vy), float(vyaw)
        self._pub.publish(m)
        self._rclpy.spin_once(self._node, timeout_sec=0.0)
        return True

    # ---- sport mode API (교재 5.2 표) --------------------------------
    def StandUp(self):        return self._send("StandUp")
    def StandDown(self):      return self._send("StandDown")
    def BalanceStand(self):   return self._send("BalanceStand")
    def Move(self, vx: float, vy: float, vyaw: float):
        return self._send("Move", vx, vy, vyaw)
    def StopMove(self):       return self._send("StopMove")
    def Reset(self):
        """시뮬레이터 전용: 월드 원점·엎드림으로 리셋(부록 B의 '리셋')."""
        return self._send("Reset")

    # ---- 상태 조회 (교재 3.2: 명령과 상태는 짝) -----------------------
    def GetMode(self) -> str:
        self._rclpy.spin_once(self._node, timeout_sec=0.05)
        return self._state.mode if self._state else ""

    def GetState(self):
        """(mode, velocity(x,y,wz), position(x,y), yaw) — 수신 전이면 None."""
        self._rclpy.spin_once(self._node, timeout_sec=0.05)
        if self._state is None:
            return None
        s = self._state
        return (s.mode, (s.velocity.x, s.velocity.y, s.velocity.z),
                (s.position.x, s.position.y), s.yaw)

    def WaitMode(self, mode: str, timeout: float = 8.0) -> bool:
        t0 = time.time()
        while time.time() - t0 < timeout:
            if self.GetMode() == mode:
                return True
            time.sleep(0.05)
        return False
