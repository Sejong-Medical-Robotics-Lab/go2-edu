"""go2_sim.kinematics — Go2 다리 기구학 (의존성 없는 순수 수학).

치수·IK는 MuJoCo 기반 선행 검증에서 FK 대조 오차 0.00 mm 로 확정한 값이며,
go2_description.urdf 의 조인트 오리진(0.1934/0.0465/0.0955/0.213/0.213)과
일치함을 확인했다.

관절 순서(JOINT_NAMES) = 실기체 DDS 12자유도 순서:
FR(0-2) · FL(3-5) · RR(6-8) · RL(9-11), 다리마다 (hip, thigh, calf)
"""
from __future__ import annotations

import math

NUM_MOTORS = 12

HIP_X = 0.1934      # base → hip 전후 오프셋
HIP_Y = 0.0465      # base → hip 좌우 오프셋
L_ABD = 0.0955      # hip → thigh 측방 오프셋
L_THIGH = 0.213
L_CALF = 0.213
FOOT_R = 0.022

STAND_H = 0.30      # 기립 시 몸통 높이 [m]
LIE_POSE = [0.0, 1.30, -2.60] * 4        # 엎드린 자세(다리 접힘)
LIE_BODY_Z = 0.12

# 다리: 이름 → (모터 시작 인덱스, x부호(앞+), y부호(왼+))
LEGS = {
    "FR": (0, +1, -1),
    "FL": (3, +1, +1),
    "RR": (6, -1, -1),
    "RL": (9, -1, +1),
}

JOINT_NAMES = [f"{leg}_{p}_joint"
               for leg in ("FR", "FL", "RR", "RL")
               for p in ("hip", "thigh", "calf")]
JOINT_INDEX = {n: i for i, n in enumerate(JOINT_NAMES)}


def leg_ik(leg: str, x: float, y: float, z_down: float):
    """고관절 기준 발끝 목표(x+앞, y+왼, z_down 아래+) → (hip, thigh, calf)."""
    _, _, ys = LEGS[leg]
    d = ys * L_ABD
    r = math.hypot(y, z_down)
    r = max(r, abs(d) + 1e-6)
    zp = math.sqrt(r * r - d * d)
    hip = math.atan2(y, z_down) - math.atan2(d, zp)

    dist = math.hypot(x, zp)
    dist = min(max(dist, 0.06), L_THIGH + L_CALF - 1e-4)
    cos_full = (2 * L_THIGH**2 - dist * dist) / (2 * L_THIGH**2)
    phi = math.acos(min(max(cos_full, -1.0), 1.0))
    calf = -(math.pi - phi)
    beta = 0.5 * (math.pi - phi)
    thigh = math.atan2(-x, zp) + beta
    return hip, thigh, calf


def stand_pose(height: float = STAND_H):
    """네 발이 각 다리 평면 바로 아래에 오는 기립 자세(12개, DDS 순서)."""
    q = [0.0] * NUM_MOTORS
    for leg, (i, _, ys) in LEGS.items():
        q[i:i + 3] = leg_ik(leg, 0.0, ys * L_ABD, height)
    return q
