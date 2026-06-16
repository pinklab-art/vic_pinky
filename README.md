# vic_pinky

<img src="/doc/img.png" width="40%" height="30%" title="vicpinky" alt="vicpinky"></img>

ROS 2 기반 차동 구동(differential drive) 자율주행 로봇 **Vic Pinky** 패키지입니다.

## 환경
- Ubuntu 24.04
- ROS 2 Jazzy

## 목차
- [PC 설정](#pc-설정)
- [ROBOT 설정](#robot-설정)
- [Vic Pinky 사용 매뉴얼](#vic-pinky-사용-매뉴얼)
  - [실행](#실행)
  - [Map building](#map-building)
  - [Navigation2](#navigation2)
  - [Web 인터페이스](#web-인터페이스)
- [시뮬레이션](#시뮬레이션)

---

# PC 설정
PC에서는 시각화(RViz) 및 시뮬레이션을 실행합니다.

### 1. Vic Pinky ROS 2 패키지 clone
```bash
mkdir -p ~/vicpinky_ws/src
cd ~/vicpinky_ws/src
git clone https://github.com/pinklab-art/vic_pinky.git
```

### 2. 의존성(dependency) 설치
```bash
cd ~/vicpinky_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 3. build
```bash
cd ~/vicpinky_ws
colcon build
```

### 4. bashrc 설정
```bash
echo 'source ~/vicpinky_ws/install/setup.bash' >> ~/.bashrc
source ~/.bashrc
```

---

# ROBOT 설정
실제 로봇(SBC)에서 실행하기 위한 설정입니다.

### 1. Vic Pinky ROS 2 패키지 clone
```bash
mkdir -p ~/vicpinky_ws/src
cd ~/vicpinky_ws/src
git clone https://github.com/pinklab-art/vic_pinky.git
```

### 2. Gazebo 패키지 삭제
실제 로봇에서는 시뮬레이션 패키지가 필요 없습니다.
```bash
cd vic_pinky
sudo rm -r vicpinky_gazebo/
```

### 3. 의존성(dependency) 설치
```bash
cd ~/vicpinky_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 4. udev 설정
rules 파일 복사
```bash
cd ~/vicpinky_ws/src/vic_pinky/doc
sudo cp ./99-vic-pinky.rules /etc/udev/rules.d/
```
udev 적용
```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

### 5. rplidar 설정
참고: <https://github.com/pinklab-art/vic_pinky/blob/main/doc/lidar_setup.md>

### 6. vicpinky 패키지 build
```bash
cd ~/vicpinky_ws
colcon build
```

### 7. bashrc 설정
```bash
echo 'source ~/vicpinky_ws/install/setup.bash' >> ~/.bashrc
source ~/.bashrc
```

---

# Vic Pinky 사용 매뉴얼

## 실행
```bash
ros2 launch vicpinky_bringup bringup.launch.xml
```

## Map building
### 1. SLAM toolbox 실행
```bash
ros2 launch vicpinky_navigation map_building.launch.xml
```
### 2. [ONLY PC] map view
```bash
ros2 launch vicpinky_navigation map_view.launch.xml
```
### 3. 키보드 조작
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
### 4. map 저장
```bash
ros2 run nav2_map_server map_saver_cli -f <map name>
```

## Navigation2
### 1. navigation2 실행
```bash
ros2 launch vicpinky_navigation bringup_launch.xml map:=<map name>
```
### 2. [ONLY PC] nav2 view
```bash
ros2 launch vicpinky_navigation nav2_view.launch.xml
```

## Web 인터페이스
브라우저에서 지도 확인, 목표 위치 지정, 초기 위치 설정, 주행 정지 등을 할 수 있는 웹 인터페이스입니다. (PC, 스마트폰, 태블릿 지원)

### 1. Navigation2 + Web (저장된 맵으로 주행)
```bash
ros2 launch vicpinky_navigation web_nav2.launch.xml map:=<map name>
```
### 2. SLAM + Web (맵 작성하며 주행)
```bash
ros2 launch vicpinky_navigation web_slam.launch.xml
```
### 3. 접속
같은 네트워크의 브라우저에서 아래 주소로 접속합니다. (기본 포트: `8080`)
```
http://<robot ip>:8080
```

#### 주요 기능
- 실시간 지도 / 로봇 위치 표시
- 지도 클릭으로 목표 위치 지정 (Nav2 주행)
- 초기 위치(2D Pose Estimate) 설정
- 주행 상태 표시 및 정지
- (SLAM 모드) 맵 초기화 및 저장

#### 옵션
| 인자 | 기본값 | 설명 |
| --- | --- | --- |
| `ip` | `0.0.0.0` | 웹 서버 바인딩 주소 |
| `port` | `8080` | 웹 서버 포트 |
| `use_sim_time` | `False` | 시뮬레이션 시간 사용 여부 |

---

# 시뮬레이션

## 실행
### 1. Gazebo 실행 및 vicpinky 스폰
```bash
ros2 launch vicpinky_bringup gazebo_bringup.launch.xml
```
### 2. (Optional) 멀티 vicpinky 스폰
```bash
ros2 launch vicpinky_bringup gazebo_multi_spwan.launch.xml namespace:=robot2 x:=12.0 y:=-16.0
```

## Map building
### 1. SLAM toolbox 실행
```bash
ros2 launch vicpinky_navigation map_building.launch.xml use_sim_time:=true
```
### 2. [ONLY PC] map view
```bash
ros2 launch vicpinky_navigation map_view.launch.xml
```
### 3. 키보드 조작
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
### 4. map 저장
```bash
ros2 run nav2_map_server map_saver_cli -f <map name>
```

## Navigation2
### 1. navigation2 실행
```bash
ros2 launch vicpinky_navigation bringup_launch.xml map:=<map name> use_sim_time:=true
```
### 2. [ONLY PC] nav2 view
```bash
ros2 launch vicpinky_navigation nav2_view.launch.xml
```
