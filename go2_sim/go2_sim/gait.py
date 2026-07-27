"""go2_sim.gait — trot(속보) 보행 생성기 (기구학 애니메이션).

교재 2.2: 대각선 짝 (FL+RR) / (FR+RL)을 번갈아 딛는 trot.
지지발은 몸통 기준 속도장 -(v + w x r_hip)을 따라 쓸리고, 유각발은
x_park 에서 이동을 마친 뒤 지면속도에 정합해 후퇴하다 touch 에 접지한다
(MuJoCo 선행 검증에서 확정한 프로파일 — 접지 제동/긁힘 없음).

베이스는 planar_move 플러그인이 기구학적으로 구동하므로, 이 모듈의 역할은
'명령 속도와 시각적으로 일치하는 다리 움직임'을 만드는 것이다. 같은 속도장을
쓰기 때문에 보폭이 실제 이동 속도와 자연스럽게 맞는다.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from .kinematics import HIP_X, HIP_Y, L_ABD, LEGS, NUM_MOTORS, leg_ik


@dataclass
class GaitParams:
    body_height: float = 0.30
    step_period: float = 0.40
    step_height: float = 0.06
    x_park: float = 0.45
    touch: float = 0.65
    max_v: float = 1.0
    max_w: float = 1.5


def _smoothstep(u: float) -> float:
    u = min(max(u, 0.0), 1.0)
    return u * u * (3 - 2 * u)


_PAIR_A = ("FL", "RR")
_HIP_POS = {leg: (xs * HIP_X, ys * (HIP_Y + L_ABD))
            for leg, (i, xs, ys) in LEGS.items()}


class GaitGenerator:
    SLEW_V = 1.5
    SLEW_W = 4.0

    def __init__(self, params: GaitParams | None = None):
        self.p = params or GaitParams()
        self.phase = 0.0
        self.vx = self.vy = self.vyaw = 0.0
        self._cmd = (0.0, 0.0, 0.0)
        self._ramp = 0.0
        self.walking = False

    def set_command(self, vx, vy, vyaw):
        p = self.p
        self._cmd = (min(max(vx, -p.max_v), p.max_v),
                     min(max(vy, -p.max_v), p.max_v),
                     min(max(vyaw, -p.max_w), p.max_w))

    def start(self):
        self.walking = True
        self.phase = 0.0
        self._ramp = 0.0

    def idle(self):
        self.walking = False
        self.vx = self.vy = self.vyaw = 0.0
        self._cmd = (0.0, 0.0, 0.0)

    def current(self):
        """슬루 적용된 현재 유효 속도 — 베이스 명령과 다리가 이 값을 공유."""
        return self.vx, self.vy, self.vyaw

    def stopped(self) -> bool:
        return (not self.walking) or (
            abs(self.vx) + abs(self.vy) + abs(self.vyaw) < 1e-3
            and sum(abs(c) for c in self._cmd) < 1e-3)

    def _foot_vel(self, leg):
        rx, ry = _HIP_POS[leg]
        return (-(self.vx - self.vyaw * ry), -(self.vy + self.vyaw * rx))

    def targets(self, dt: float):
        p = self.p
        for name, lim in (("vx", self.SLEW_V * dt), ("vy", self.SLEW_V * dt),
                          ("vyaw", self.SLEW_W * dt)):
            cur = getattr(self, name)
            tgt = self._cmd[("vx", "vy", "vyaw").index(name)]
            setattr(self, name, cur + min(max(tgt - cur, -lim), lim))

        if self.stopped():
            self.phase = 0.0
            self._ramp = 0.0
        else:
            self.phase = (self.phase + dt / p.step_period) % 1.0
            self._ramp = min(self._ramp + dt / 0.6, 1.0)

        q = [0.0] * NUM_MOTORS
        half_T = 0.5 * p.step_period
        for leg, (i, xs, ys) in LEGS.items():
            ph = self.phase if leg in _PAIR_A else (self.phase + 0.5) % 1.0
            swing = ph < 0.5 and not self.stopped()
            u = ph / 0.5 if swing else (ph - 0.5) / 0.5

            fvx, fvy = self._foot_vel(leg)
            hx = 0.5 * fvx * half_T * self._ramp
            hy = 0.5 * fvy * half_T * self._ramp
            if swing:
                px = -hx * (1 + 2 * (1 - p.x_park))
                py = -hy * (1 + 2 * (1 - p.x_park))
                su = _smoothstep(min(u / p.x_park, 1.0))
                x = hx + (px - hx) * su
                y = hy + (py - hy) * su
                if u > p.x_park:
                    x += fvx * (u - p.x_park) * half_T
                    y += fvy * (u - p.x_park) * half_T
                z = p.body_height - self._ramp * p.step_height * math.sin(
                    math.pi * min(u / p.touch, 1.0)) if u <= p.touch \
                    else p.body_height
            else:
                x = hx * (2 * u - 1)
                y = hy * (2 * u - 1)
                z = p.body_height

            q[i:i + 3] = leg_ik(leg, x, ys * L_ABD + y, z)
        return q
