# vic_pinky
<img src="/doc/image.png" width="40%" height="30%" title="vicpinky" alt="vicpinky"></img>

# ROBOT 설정
## 환경
- ubuntu 22.04
- ros2 humble
## 1. vicpinky ROS2 pkg clone
```
mkdir -p ~/vicpinky_ws/src
cd ~/vicpinky_ws/src
git clone https://github.com/pinklab-art/vic_pinky.git
````
## 2. dependence 설치
```
cd ~/vicpinky_ws
rosdep install --from-paths src --ignore-src -r -y
```
## 3. udev 설정
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
## 4. rplidar 설정
참고: <https://github.com/pinklab-art/vicpinky_ws/blob/main/doc/lidar_setup.md>

## 5. vicpinky_ws pkg build
```
cd ~/vicpinky_ws
colcon build
```
## 8. set bahsrc
```
echo 'source ~/vicpinky_ws/install/setup.bash' >> ~/.bashrc
source ~/.bashrc
```
# Vic Pinky brinup

```
ros2 launch vicpinky_bringup bringup.launch.xml
```
