"""Go2 교육용 시뮬레이터 launch (교재 4.3의 'sim 런치파일').

    ros2 launch go2_gazebo sim.launch.py            # Gazebo GUI + 로봇 + 관제 노드
    ros2 launch go2_gazebo sim.launch.py gui:=false # 헤드리스(WSL2 저성능 대안)
    ros2 launch go2_gazebo sim.launch.py max_v:=0.5 # 멘토 속도 상한 (교재 5.1)

gzserver 가 /clock 을 발행하고 모든 노드가 use_sim_time 으로 이를 따른다
(교재 4.2 '두 개의 시계'). Gazebo GUI 의 ▶/⏸·리셋·RTF 표시는 교재 4.2 그대로.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_gazebo_ros = get_package_share_directory("gazebo_ros")
    pkg_self = get_package_share_directory("go2_gazebo")

    world = os.path.join(pkg_self, "worlds", "mission.world")
    urdf = os.path.join(pkg_self, "urdf", "go2_sim.urdf")
    with open(urdf, "r") as f:
        robot_description = f.read()

    gui = LaunchConfiguration("gui")
    max_v = LaunchConfiguration("max_v")
    max_w = LaunchConfiguration("max_w")

    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("max_v", default_value="1.0",
                              description="속도 상한 [m/s] — 멘토 설정"),
        DeclareLaunchArgument("max_w", default_value="1.5",
                              description="회전 상한 [rad/s]"),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, "launch", "gzserver.launch.py")),
            launch_arguments={"world": world, "verbose": "false"}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, "launch", "gzclient.launch.py")),
            condition=IfCondition(gui),
        ),

        Node(package="robot_state_publisher", executable="robot_state_publisher",
             name="robot_state_publisher", output="screen",
             parameters=[{"use_sim_time": True,
                          "robot_description": robot_description}]),

        Node(package="gazebo_ros", executable="spawn_entity.py",
             name="spawn_go2", output="screen",
             arguments=["-entity", "go2", "-topic", "robot_description",
                        "-z", "0.45"]),

        Node(package="go2_sim", executable="supervisor",
             name="go2_supervisor", output="screen",
             parameters=[{"use_sim_time": True,
                          "max_v": max_v, "max_w": max_w}]),
    ])
