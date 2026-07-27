#!/usr/bin/env python3
"""go2_gazebo/urdf/go2_sim.urdf 생성기.

go2_description.urdf(원본 무수정)를 읽어 시뮬 운용에 맞게 변환한다:
  1) 다리·머리 링크의 <collision> 제거 — 관절을 기구학적으로 구동
     (joint_pose_trajectory)하므로 다리 접촉이 물리와 싸우지 않게 한다.
     충돌은 base(장애물 차단)와 4개 foot(지면 지지)만 유지.
  2) 발 마찰을 낮춤(mu 0.05) — 지지 중 스케이팅 허용(베이스는 planar_move 가
     기구학적으로 구동).
  3) 직립 lidar_link 추가(실기체 radar 장착 위치, 자세만 수평) + 센서 플러그인.
  4) gazebo_ros 플러그인 블록: planar_move / joint_state_publisher /
     joint_pose_trajectory / imu_sensor / ray_sensor(PointCloud2).

재생성:  python3 tools/gen_sim_urdf.py
"""
import math
import os
import xml.etree.ElementTree as ET

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
SRC = os.path.join(ROOT, "go2_description", "urdf", "go2_description.urdf")
DST = os.path.join(ROOT, "go2_gazebo", "urdf", "go2_sim.urdf")

KEEP_COLLISION = {"base", "FR_foot", "FL_foot", "RR_foot", "RL_foot"}
FEET = ["FR_foot", "FL_foot", "RR_foot", "RL_foot"]
JOINTS_12 = [f"{leg}_{p}_joint" for leg in ("FR", "FL", "RR", "RL")
             for p in ("hip", "thigh", "calf")]

LIDAR_XYZ = "0.28945 0 0.0"       # 실기체 radar 장착 위치(x,y), 자세는 수평
LIDAR_H_SAMPLES = 120
LIDAR_V_SAMPLES = 8
LIDAR_V_MIN = math.radians(-70)
LIDAR_V_MAX = math.radians(25)


def E(tag, text=None, **attr):
    e = ET.Element(tag, {k: str(v) for k, v in attr.items()})
    if text is not None:
        e.text = str(text)
    return e


def sub(parent, tag, text=None, **attr):
    e = E(tag, text, **attr)
    parent.append(e)
    return e


def main():
    tree = ET.parse(SRC)
    robot = tree.getroot()

    # 1) 충돌 정리
    removed = 0
    for link in robot.findall("link"):
        if link.get("name") in KEEP_COLLISION:
            continue
        for col in link.findall("collision"):
            link.remove(col)
            removed += 1

    # 3) lidar_link (질량 미미한 고정 링크)
    lid = sub(robot, "link", name="lidar_link")
    inr = sub(lid, "inertial")
    sub(inr, "origin", xyz="0 0 0", rpy="0 0 0")
    sub(inr, "mass", value="0.001")
    sub(inr, "inertia", ixx="1e-6", ixy="0", ixz="0",
        iyy="1e-6", iyz="0", izz="1e-6")
    lj = sub(robot, "joint", name="lidar_joint", type="fixed")
    sub(lj, "origin", xyz=LIDAR_XYZ, rpy="0 0 0")
    sub(lj, "parent", link="base")
    sub(lj, "child", link="lidar_link")

    # 2) 발 마찰·베이스 감쇠
    for f in FEET:
        g = sub(robot, "gazebo", reference=f)
        sub(g, "mu1", "0.05")
        sub(g, "mu2", "0.05")
        sub(g, "kp", "100000")
        sub(g, "kd", "100")
    gb = sub(robot, "gazebo", reference="base")
    sub(gb, "dampingFactor", "0.01")

    # 4) 모델 플러그인들 (전역 <gazebo>)
    gz = sub(robot, "gazebo")

    p = sub(gz, "plugin", name="planar_move",
            filename="libgazebo_ros_planar_move.so")
    ros = sub(p, "ros")
    sub(ros, "remapping", "cmd_vel:=/go2/_base_cmd")
    sub(ros, "remapping", "odom:=/odom_gt")
    sub(p, "update_rate", "100")
    sub(p, "publish_rate", "20")
    sub(p, "publish_odom", "true")
    sub(p, "publish_odom_tf", "false")
    sub(p, "odometry_frame", "odom_gt")
    sub(p, "robot_base_frame", "base")

    p = sub(gz, "plugin", name="joint_states",
            filename="libgazebo_ros_joint_state_publisher.so")
    sub(p, "update_rate", "50")
    for j in JOINTS_12:
        sub(p, "joint_name", j)

    p = sub(gz, "plugin", name="joint_pose_trajectory",
            filename="libgazebo_ros_joint_pose_trajectory.so")
    ros = sub(p, "ros")
    sub(ros, "remapping", "set_joint_trajectory:=/go2/set_joint_trajectory")
    sub(p, "update_rate", "100")

    # IMU (기존 imu 링크에 부착)
    gi = sub(robot, "gazebo", reference="imu")
    sen = sub(gi, "sensor", name="imu_sensor", type="imu")
    sub(sen, "always_on", "true")
    sub(sen, "update_rate", "100")
    p = sub(sen, "plugin", name="imu_plugin",
            filename="libgazebo_ros_imu_sensor.so")
    ros = sub(p, "ros")
    sub(ros, "remapping", "~/out:=/go2/imu")
    sub(p, "frame_name", "imu")
    sub(p, "initial_orientation_as_reference", "false")

    # LiDAR (ray → PointCloud2)
    gl = sub(robot, "gazebo", reference="lidar_link")
    sen = sub(gl, "sensor", name="l1_lidar", type="ray")
    sub(sen, "always_on", "true")
    sub(sen, "update_rate", "10")
    sub(sen, "visualize", "false")
    ray = sub(sen, "ray")
    scan = sub(ray, "scan")
    h = sub(scan, "horizontal")
    sub(h, "samples", LIDAR_H_SAMPLES)
    sub(h, "resolution", "1")
    sub(h, "min_angle", f"{-math.pi:.6f}")
    sub(h, "max_angle", f"{math.pi:.6f}")
    v = sub(scan, "vertical")
    sub(v, "samples", LIDAR_V_SAMPLES)
    sub(v, "resolution", "1")
    sub(v, "min_angle", f"{LIDAR_V_MIN:.6f}")
    sub(v, "max_angle", f"{LIDAR_V_MAX:.6f}")
    rng = sub(ray, "range")
    sub(rng, "min", "0.15")
    sub(rng, "max", "12.0")
    sub(rng, "resolution", "0.01")
    p = sub(sen, "plugin", name="lidar_plugin",
            filename="libgazebo_ros_ray_sensor.so")
    ros = sub(p, "ros")
    sub(ros, "remapping", "~/out:=/go2/pointcloud")
    sub(p, "output_type", "sensor_msgs/PointCloud2")
    sub(p, "frame_name", "lidar_link")

    ET.indent(tree, space="  ")
    header = ("<!-- 자동 생성 파일: tools/gen_sim_urdf.py 로 재생성. 직접 수정 금지.\n"
              "     원본: go2_description/urdf/go2_description.urdf (BSD-3, 무수정 보존)\n"
              f"     변경: 다리 collision 제거({removed}개), lidar_link 추가, "
              "gazebo 플러그인 블록 추가 -->\n")
    body = ET.tostring(robot, encoding="unicode")
    with open(DST, "w") as f:
        f.write('<?xml version="1.0"?>\n' + header + body + "\n")
    print(f"생성: {os.path.relpath(DST, ROOT)}  (collision 제거 {removed}개, "
          f"라이다 {LIDAR_H_SAMPLES}x{LIDAR_V_SAMPLES}선)")


if __name__ == "__main__":
    main()
