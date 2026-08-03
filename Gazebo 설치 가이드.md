# 1. 실습 환경

본 가이드는 다음 환경을 기준으로 작성되었습니다.

- Ubuntu 22.04
- ROS2 Humble
- Gazebo Classic

ROS2 Humble 설치는 공통교육 교안을 참고하여 미리 완료합니다.

본 실습에서는 Go2의 Gazebo 시뮬레이션을 위해 아래 저장소를 사용합니다.

[unitree-go2-ros2 저장소](https://github.com/anujjain-dev/unitree-go2-ros2)

---

# 2. Go2 시뮬레이션 설치

## 2.1 터미널 열기

Ubuntu에서 `Ctrl + Alt + T`를 눌러 터미널을 엽니다.

먼저 ROS2가 정상적으로 설치되어 있는지 확인합니다.

```bash
source /opt/ros/humble/setup.bash
echo $ROS_DISTRO
```

정상적으로 설치되어 있다면 다음과 같이 출력됩니다.

```text
humble
```

---

## 2.2 필요한 패키지 설치

아래 명령어를 그대로 복사하여 실행합니다.

```bash
sudo apt update

sudo apt install -y \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-gazebo-ros2-control \
  ros-humble-xacro \
  ros-humble-robot-localization \
  ros-humble-ros2-controllers \
  ros-humble-ros2-control \
  ros-humble-velodyne \
  ros-humble-velodyne-gazebo-plugins \
  ros-humble-velodyne-description \
  python3-rosdep \
  python3-colcon-common-extensions \
  git
```

설치가 완료될 때까지 기다립니다.

---

## 2.3 rosdep 설정

다음 명령어를 실행합니다.

```bash
sudo rosdep init
```

이미 rosdep이 초기화되어 있다는 메시지가 출력되면 오류가 아니므로 그대로 다음 단계로 넘어갑니다.

이후 rosdep 정보를 업데이트합니다.

```bash
rosdep update
```

---

## 2.4 Go2 작업공간 생성

Go2 시뮬레이션 패키지를 설치할 작업공간을 생성합니다.

```bash
mkdir -p ~/go2_ws/src
cd ~/go2_ws/src
```

Go2 시뮬레이션 저장소를 내려받습니다.

```bash
git clone https://github.com/anujjain-dev/unitree-go2-ros2.git
```

---

## 2.5 의존성 설치

작업공간으로 이동한 뒤, 패키지 실행에 필요한 의존성을 설치합니다.

```bash
cd ~/go2_ws
rosdep install --from-paths src --ignore-src -r -y
```

---

## 2.6 빌드

다음 명령어로 Go2 작업공간을 빌드합니다.

```bash
cd ~/go2_ws
colcon build
```

빌드가 완료되면 작업공간의 환경 설정 파일을 적용합니다.

```bash
source ~/go2_ws/install/setup.bash
```

> **주의**
>
> 새로운 터미널을 열 때마다 Go2 패키지를 사용하기 전에 다음 명령어를 다시 실행해야 합니다.
>
> ```bash
> source ~/go2_ws/install/setup.bash
> ```

---

# 3. 설치 완료 확인

다음 명령어로 Go2 Gazebo 시뮬레이션을 실행합니다.

```bash
source ~/go2_ws/install/setup.bash
ros2 launch go2_config gazebo.launch.py
```

정상적으로 실행되면 다음 항목을 확인합니다.

- Gazebo가 정상적으로 열린다.
- Gazebo 화면에 Go2 로봇이 나타난다.
- 로봇이 바닥에 정상적으로 서 있다.
- Gazebo 화면의 RTF가 약 **0.8 이상**이다.
