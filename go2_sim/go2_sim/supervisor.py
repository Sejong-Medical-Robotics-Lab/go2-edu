#!/usr/bin/env python3
"""go2_sim supervisor — Gazebo 위의 sport mode 관제 노드.

역할 분담:
  Gazebo 플러그인  : 물리·베이스 이동(planar_move)·센서(ray/imu)·joint_states
  이 노드(core.py) : sport mode FSM · 명령 게이트/워치독 · trot 다리 애니메이션
                     · 오도메트리(드리프트 포함) · 상태 토픽/tf

발행: /go2/sportmodestate(20Hz) /go2/lowstate(50Hz, /joint_states 재구성)
      /odom + tf odom→base(20Hz)
      /go2/_base_cmd(Twist→planar_move) /go2/set_joint_trajectory(다리)
구독: /cmd_vel /go2/api /joint_states /go2/imu
"""
from __future__ import annotations

import math

import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from tf2_ros import TransformBroadcaster
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from go2_edu_interfaces.msg import (ApiCommand, IMUState, LowState,
                                    MotorState, SportModeState)

from .core import SupervisorCore
from .kinematics import JOINT_NAMES

TICK_HZ = 50.0
STATE_HZ = 20.0


class Supervisor(Node):
    def __init__(self):
        super().__init__("go2_supervisor")
        self.declare_parameter("max_v", 1.0)
        self.declare_parameter("max_w", 1.5)
        max_v = float(self.get_parameter("max_v").value)
        max_w = float(self.get_parameter("max_w").value)
        self.core = SupervisorCore(max_v=max_v, max_w=max_w)
        self.get_logger().info(
            f"sport mode 관제 시작 (max_v={max_v}, max_w={max_w}) — "
            "초기 자세: 엎드림(StandUp 필요)")

        self.pub_base = self.create_publisher(Twist, "/go2/_base_cmd", 10)
        self.pub_traj = self.create_publisher(
            JointTrajectory, "/go2/set_joint_trajectory", 10)
        self.pub_sport = self.create_publisher(
            SportModeState, "/go2/sportmodestate", 10)
        self.pub_low = self.create_publisher(LowState, "/go2/lowstate", 10)
        self.pub_odom = self.create_publisher(Odometry, "/odom", 10)
        self.tf = TransformBroadcaster(self)

        self.create_subscription(Twist, "/cmd_vel", self._on_cmd_vel, 10)
        self.create_subscription(ApiCommand, "/go2/api", self._on_api, 10)
        self.create_subscription(JointState, "/joint_states",
                                 self._on_joint_states, 20)
        self.create_subscription(Imu, "/go2/imu", self._on_imu, 20)

        self._imu = None
        self._low_tick = 0
        self._last = None
        self._out = None
        self.create_timer(1.0 / TICK_HZ, self._tick)
        self.create_timer(1.0 / STATE_HZ, self._pub_state)

    # ---- 시간 ----------------------------------------------------------
    def _now(self) -> float:
        t = self.get_clock().now().seconds_nanoseconds()
        return t[0] + t[1] * 1e-9

    # ---- 구독 ----------------------------------------------------------
    def _on_cmd_vel(self, m: Twist):
        ok, msg = self.core.cmd_vel(m.linear.x, m.linear.y, m.angular.z,
                                    self._now())
        if not ok:
            self.get_logger().warn(msg, throttle_duration_sec=2.0)

    def _on_api(self, m: ApiCommand):
        ok, msg = self.core.api(m.name, m.vx, m.vy, m.vyaw, self._now())
        if not ok:
            self.get_logger().warn(msg)

    def _on_imu(self, m: Imu):
        self._imu = m

    def _on_joint_states(self, m: JointState):
        """/joint_states → LowState(실기체 DDS 순서로 재정렬) 50Hz."""
        self._low_tick += 1
        idx = {}
        for k, name in enumerate(m.name):
            idx[name] = k
        out = LowState()
        out.tick = self._low_tick
        for name in JOINT_NAMES:
            ms = MotorState()
            k = idx.get(name)
            if k is not None:
                ms.q = float(m.position[k]) if k < len(m.position) else 0.0
                ms.dq = float(m.velocity[k]) if k < len(m.velocity) else 0.0
                ms.tau_est = float(m.effort[k]) if k < len(m.effort) else 0.0
                ms.temperature = 25.0 + 2.0 * abs(ms.dq)
            out.motor_state.append(ms)
        imu = IMUState()
        if self._imu is not None:
            q = self._imu.orientation
            imu.quaternion = [float(q.w), float(q.x), float(q.y), float(q.z)]
            g = self._imu.angular_velocity
            imu.gyroscope = [float(g.x), float(g.y), float(g.z)]
            a = self._imu.linear_acceleration
            imu.accelerometer = [float(a.x), float(a.y), float(a.z)]
            sy = 2 * (q.w * q.z + q.x * q.y)
            cy = 1 - 2 * (q.y * q.y + q.z * q.z)
            imu.rpy = [
                math.atan2(2 * (q.w * q.x + q.y * q.z),
                           1 - 2 * (q.x * q.x + q.y * q.y)),
                math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x)))),
                math.atan2(sy, cy)]
        out.imu_state = imu
        self.pub_low.publish(out)

    # ---- 50Hz 관제 ------------------------------------------------------
    def _tick(self):
        t = self._now()
        dt = 1.0 / TICK_HZ if self._last is None else max(t - self._last, 0.0)
        self._last = t
        if dt == 0.0:                       # /clock 일시정지 중 (교재 4.2)
            return
        self._out = self.core.tick(t, min(dt, 0.1))

        tw = Twist()
        tw.linear.x, tw.linear.y = self._out["base_cmd"][0], self._out["base_cmd"][1]
        tw.angular.z = self._out["base_cmd"][2]
        self.pub_base.publish(tw)

        jt = JointTrajectory()
        jt.joint_names = list(JOINT_NAMES)
        pt = JointTrajectoryPoint()
        pt.positions = [float(v) for v in self._out["targets"]]
        pt.time_from_start = Duration(nanosec=int(1e9 / TICK_HZ))
        jt.points.append(pt)
        self.pub_traj.publish(jt)

    # ---- 20Hz 상태/odom/tf ---------------------------------------------
    def _pub_state(self):
        if self._out is None:
            return
        stamp = self.get_clock().now().to_msg()
        x, y, yaw = self.core.odom
        vx, vy, wz = self._out["base_cmd"]

        s = SportModeState()
        s.header.stamp = stamp
        s.header.frame_id = "odom"
        s.mode, s.error = self._out["mode"], self._out["error"]
        s.velocity.x, s.velocity.y, s.velocity.z = vx, vy, wz
        s.position.x, s.position.y, s.position.z = x, y, 0.0
        s.yaw = yaw
        self.pub_sport.publish(s)

        od = Odometry()
        od.header.stamp = stamp
        od.header.frame_id = "odom"
        od.child_frame_id = "base"
        od.pose.pose.position.x, od.pose.pose.position.y = x, y
        od.pose.pose.position.z = self._out["body_z"]
        od.pose.pose.orientation.w = math.cos(yaw / 2)
        od.pose.pose.orientation.z = math.sin(yaw / 2)
        od.twist.twist.linear.x, od.twist.twist.linear.y = vx, vy
        od.twist.twist.angular.z = wz
        self.pub_odom.publish(od)

        T = TransformStamped()
        T.header.stamp = stamp
        T.header.frame_id = "odom"
        T.child_frame_id = "base"
        T.transform.translation.x, T.transform.translation.y = x, y
        T.transform.translation.z = self._out["body_z"]
        T.transform.rotation.w = math.cos(yaw / 2)
        T.transform.rotation.z = math.sin(yaw / 2)
        self.tf.sendTransform(T)


def main(args=None):
    rclpy.init(args=args)
    node = Supervisor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == "__main__":
    main()
