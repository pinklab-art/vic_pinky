# vic_pinky
<img src="/doc/image.png" width="40%" height="30%" title="vicpinky" alt="vicpinky"></img>

# PC 설정
## 환경
- ubuntu 24.04
- ros2 jazzy
## 1. Vic Pinky ROS2 pkg clone
```
mkdir -p ~/pinky_violet/src
cd ~/pinky_violet/src
git clone https://github.com/pinklab-art/pinky_violet.git
```
## 2. dependence 설치
```
cd ~/pinky_violet
rosdep install --from-paths src --ignore-src -r -y
```
## 3. build
```
cd ~/pinky_violet
colcon build
```

# ROBOT 설정
## 환경
- ubuntu 24.04
- ros2 jazzy
## 1. Vic pinky ROS2 pkg clone
```
mkdir -p ~/vicpinky_ws/src
cd ~/vicpinky_ws/src
git clone https://github.com/pinklab-art/vic_pinky.git
````
## 2. 가제보 패키지 삭제
```
cd vic_pinky
sudo rm -r vicpinky_gazebo/
````
## 3. dependence 설치
```
cd ~/vicpinky_ws
rosdep install --from-paths src --ignore-src -r -y
```
## 4. udev 설정
#### rulse 파일 복사
```
cd ~/vicpinky_ws/src/vic_pinky/doc
sudo cp ./99-vic-pinky.rules /etc/udev/rules.d/
```
#### udev 적용
```
sudo udevadm control --reload-rules
sudo udevadm trigger
```
## 5. rplidar 설정
참고: <[https://github.com/pinklab-art/vicpinky_ws/blob/main/doc/lidar_setup.md](https://github.com/pinklab-art/vic_pinky/blob/main/doc/lidar_setup.md)>

## 6. vicpinky pkg build
```
cd ~/vicpinky_ws
colcon build
```
## 7. set bahsrc
```
echo 'source ~/vicpinky_ws/install/setup.bash' >> ~/.bashrc
source ~/.bashrc
```


# Vic Pinky 사용 매뉴얼

## Vic pinky 실행
```
ros2 launch vicpinky_bringup bringup.launch.xml
```

## Map building
#### launch slam toolbox
```
ros2 launch vicpinky_navigation map_building.launch.xml
```
#### [ONLY PC] map view 
```
ros2 launch vicpinky_navigation map_view.launch.xml
```
#### robot keyborad control
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard 
```
#### map save 
```
ros2 run nav2_map_server map_saver_cli -f <map name>
```

## Navigation2 
#### launch navigation2
```
ros2 launch vicpinky_navigation bringup_launch.xml map:=<map name>
```

#### [ONLY PC] nav2 view
```
ros2 launch vicpinky_navigation nav2_view.launch.xml
```

# 시뮬레이션
## Vic pinky gazebo 실행
#### 가제보 실행 및 vicpinky 스폰
```
ros2 launch vicpinky_bringup gazebo_bringup.launch.xml
```
#### (Optional) 멀티 vicpinky 스폰
```
ros2 launch vicpinky_bringup gazebo_multi_spwan.launch.xml namespace:=robot2 x:=12.0 y:=-16.0
```
## Map building
#### launch slam toolbox
```
ros2 launch vicpinky_navigation map_building.launch.xml use_sim_time:=true
```
#### [ONLY PC] map view 
```
ros2 launch vicpinky_navigation map_view.launch.xml
```
#### robot keyborad control
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard 
```
#### map save 
```
ros2 run nav2_map_server map_saver_cli -f <map name>
```

## Navigation2 
#### launch navigation2
```
ros2 launch vicpinky_navigation bringup_launch.xml map:=<map name> use_sim_time:=true
```

#### [ONLY PC] nav2 view
```
ros2 launch vicpinky_navigation nav2_view.launch.xml
```
