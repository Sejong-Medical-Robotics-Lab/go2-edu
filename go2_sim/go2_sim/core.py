"""go2_sim.core — 관제 코어 (ROS 무관 순수 로직 → 오프라인 단위검증 대상).

sport mode FSM :  lying → standing_up → standing ↔ moving,
                  StandDown → lying_down → lying,  Reset → lying(오도메트리 0)
명령 게이트    :  기립 상태에서만 이동 명령 유효 (교재 2.3 / 5.4)
명령 유지 방식 :  API Move = 1회 유지형(StopMove 까지),
                  /cmd_vel = 0.5초 워치독형(계속 보내야 유지) — 교재 5.3
오도메트리     :  게이트 통과 속도의 적분 + 의도적 결함(스케일 0.985,
                  요 바이어스 0.002 rad/s) → 드리프트 관찰 미션 성립(교재 3.2)
                  (베이스는 planar_move 가 명령 속도 그대로 구동하므로
                   실제 이동 ≈ 명령 적분이고, 오도메트리는 그보다 살짝 어긋난다)

supervisor.py 는 이 코어를 ROS 토픽에 얇게 배선만 한다.
"""
from __future__ import annotations

from .gait import GaitGenerator, GaitParams
from .kinematics import LIE_BODY_Z, LIE_POSE, STAND_H, stand_pose

LYING = "lying"
STANDING_UP = "standing_up"
STANDING = "standing"
MOVING = "moving"
LYING_DOWN = "lying_down"

TRANSITION_TIME = 1.2
CMD_VEL_TIMEOUT = 0.5


def _lerp(a, b, u):
    return [x + (y - x) * u for x, y in zip(a, b)]


class SupervisorCore:
    def __init__(self, max_v: float = 1.0, max_w: float = 1.5):
        self.gait = GaitGenerator(GaitParams(max_v=max_v, max_w=max_w))
        self.mode = LYING
        self.error = ""
        self._targets = list(LIE_POSE)
        self._trans_from = list(LIE_POSE)
        self._trans_to = list(LIE_POSE)
        self._trans_t = 0.0
        self._cmdvel_stamp = -1e9
        self._cmdvel_active = False
        self.odom = [0.0, 0.0, 0.0]          # x, y, yaw (odom 프레임 추정)
        self._ODO_SCALE = 0.985
        self._ODO_YAW_BIAS = 0.002

    # ---- sport mode API ------------------------------------------------
    def api(self, name: str, vx=0.0, vy=0.0, vyaw=0.0, t: float = 0.0):
        name = name.strip()
        if name == "StandUp":
            if self.mode != LYING:
                return self._rej(f"StandUp 무시: 현재 {self.mode}")
            self._begin(STANDING_UP, stand_pose())
            return True, ""
        if name == "StandDown":
            if self.mode not in (STANDING, MOVING):
                return self._rej(f"StandDown 무시: 현재 {self.mode}")
            self.gait.idle()
            self._begin(LYING_DOWN, list(LIE_POSE))
            return True, ""
        if name == "BalanceStand":
            if self.mode == MOVING:
                return self.api("StopMove")
            if self.mode != STANDING:
                return self._rej(f"BalanceStand 무시: 현재 {self.mode}")
            return True, ""
        if name == "Move":
            return self.move(vx, vy, vyaw, t, from_topic=False)
        if name == "StopMove":
            if self.mode == MOVING:
                self.gait.set_command(0.0, 0.0, 0.0)
                self._cmdvel_active = False
            return True, ""
        if name == "Reset":
            self.reset()
            return True, ""
        return self._rej(f"알 수 없는 명령: {name}")

    def move(self, vx, vy, vyaw, t: float, from_topic: bool):
        if self.mode not in (STANDING, MOVING):
            return self._rej(
                f"Move 무시: 기립 상태가 아님 (현재 {self.mode}) — "
                "StandUp이 먼저입니다 (교재 5.4)")
        if self.mode == STANDING:
            self.gait.start()
            self.mode = MOVING
        self.gait.set_command(vx, vy, vyaw)
        if from_topic:
            self._cmdvel_stamp = t
            self._cmdvel_active = True
        else:
            self._cmdvel_active = False       # API 경로 = 유지형
        return True, ""

    def cmd_vel(self, vx, vy, vyaw, t: float):
        return self.move(vx, vy, vyaw, t, from_topic=True)

    def reset(self):
        self.gait.idle()
        self.mode = LYING
        self.error = ""
        self._targets = list(LIE_POSE)
        self._trans_from = list(LIE_POSE)
        self.odom = [0.0, 0.0, 0.0]

    def _rej(self, msg):
        return False, msg

    def _begin(self, mode, target):
        self.gait.idle()
        self.mode = mode
        self._trans_t = 0.0
        self._trans_from = list(self._targets)   # 기구학 구동 ⇒ 실측=직전 명령
        self._trans_to = list(target)

    # ---- 주기 갱신 -----------------------------------------------------
    def tick(self, t: float, dt: float) -> dict:
        # 워치독 (교재 5.3)
        if (self.mode == MOVING and self._cmdvel_active
                and t - self._cmdvel_stamp > CMD_VEL_TIMEOUT):
            self.gait.set_command(0.0, 0.0, 0.0)
            self._cmdvel_active = False

        if self.mode in (STANDING_UP, LYING_DOWN):
            self._trans_t += dt
            u = min(self._trans_t / TRANSITION_TIME, 1.0)
            u = u * u * (3 - 2 * u)
            self._targets = _lerp(self._trans_from, self._trans_to, u)
            if self._trans_t >= TRANSITION_TIME:
                self.mode = STANDING if self.mode == STANDING_UP else LYING
        elif self.mode in (STANDING, MOVING):
            self._targets = self.gait.targets(dt)
            if self.mode == MOVING and self.gait.stopped():
                self.gait.idle()
                self.mode = STANDING

        # 베이스 속도 명령(기립·이동에서만; 그 외 0)
        if self.mode in (STANDING, MOVING):
            bvx, bvy, bw = self.gait.current()
        else:
            bvx = bvy = bw = 0.0

        # 오도메트리 적분(의도적 결함 포함)
        import math
        self.odom[2] += (bw + self._ODO_YAW_BIAS * (1 if self.mode == MOVING
                                                    else 0)) * dt
        c, s = math.cos(self.odom[2]), math.sin(self.odom[2])
        self.odom[0] += self._ODO_SCALE * (c * bvx - s * bvy) * dt
        self.odom[1] += self._ODO_SCALE * (s * bvx + c * bvy) * dt

        return dict(targets=list(self._targets),
                    base_cmd=(bvx, bvy, bw),
                    mode=self.mode, error=self.error,
                    body_z=self.body_z())

    def body_z(self) -> float:
        if self.mode in (STANDING, MOVING):
            return STAND_H
        if self.mode == LYING:
            return LIE_BODY_Z
        u = min(self._trans_t / TRANSITION_TIME, 1.0)
        if self.mode == STANDING_UP:
            return LIE_BODY_Z + (STAND_H - LIE_BODY_Z) * u
        return STAND_H + (LIE_BODY_Z - STAND_H) * u
