# RPLIDAR C1 SETUP

### lidar 설정 전 udev 설정  후 진행
## 1. lidar 설치- https://github.com/Slamtec/sllidar_ros2 참고
```
cd ~/vicpinky_ws/src
git clone https://github.com/Slamtec/sllidar_ros2.git
cd ~/vicpinky_ws/
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --symlink-install
```

## 2. 설정 터미널 확인 
```
ls -al /dev/rplidar
```
```
lrwxrwxrwx 1 root root 7 Jun 17 14:06 /dev/rplidar -> ttyUSB1
```
위와 같이 출력이 안되었다면 udev 설정
```
ls -al /dev/rplidar
```
## 3.launch 파일 수정 (15, 17 line)
```
 sudo nano ~/vicpinky_ws/src/sllidar_ros2/launch/sllidar_c1_launch.py
 ```

####  전 
```
serial_port = LaunchConfiguration('serial_port', default='/dev/USB0')

frame_id = LaunchConfiguration('frame_id', default='laser')
```

####  후 
```
serial_port = LaunchConfiguration('serial_port', default='/dev/rplidar')

frame_id = LaunchConfiguration('frame_id', default='laser_link')
```
## 4. Lidar 실행 및 터미널 확인
```
ros2 launch sllidar_ros2 sllidar_c1_launch.py
```
```
ros2 topic echo /scan
```
