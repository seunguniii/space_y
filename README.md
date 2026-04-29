# PX4-Gazebo Autoland System 설치 및 실행 가이드

본 문서는 **LiDAR 및 ArUco 마커 기반 자동 착륙 시스템 (Autoland Module)** 을 구축하고 실행하는 전 과정을 정리한 것입니다.  
ROS2 Humble + PX4 SITL + Gazebo Harmonic 환경 기준입니다.

---

# 🧩 설치 순서

1. **Ubuntu 22.04 설치**
2. **ROS2 Humble 설치**  
   🔗 https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html
3. **PX4 main branch 설치**
   ```bash
   git clone https://github.com/PX4/PX4-Autopilot.git --recursive
   cd PX4-Autopilot
   make px4_sitl
   ```
   잘 실행되는지 확인
4. **Micro XRCE-DDS Agent 설치**  
   ```bash
   git clone -b v2.4.3 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
   cd Micro-XRCE-DDS-Agent
   mkdir build
   cd build
   cmake ..
   make
   sudo make install
   sudo ldconfig /usr/local/lib/
   MicroXRCEAgent udp4 -p 8888
   잘 실행되는지 확인
6. **QGroundControl 설치**  
   🔗 https://docs.qgroundcontrol.com/master/en/qgc-user-guide/getting_started/download_and_install.html
7. **OpenCV 4.5.4에 GStreamer 설치 여부 확인 (중요)**  
   ```python
   import cv2
   print(cv2.getBuildInformation())
   ```
   ✅ `GStreamer: YES` 확인 필수
8. **ros-gz-bridge 설치 (from source)**  
   ```bash
   sudo apt install ros-humble-ros-gzharmonic
   ```
9. **px4_msgs workspace 설치 및 빌드**  
   ```bash
   mkdir -p ~/px4_ros_ws/src/
   cd ~/px4_ros_ws/src/
   git clone https://github.com/PX4/px4_msgs.git
   git clone https://github.com/PX4/px4_ros_com.git
   ```
10. **LiDAR 및 ArUco 마커 기반 정밀 착륙 시뮬레이션 문서** 참고하여 자동착륙 모듈 실행

> 💡 **필요한 워크스페이스 3개**
> - `px4_ros_ws`
> - `ros-gz-bridge`
> - `ws_KRAC`  
> 이 3개가 모두 있어야 Autoland 모듈이 정상적으로 작동합니다.

---

# 🛫 코드 실행 순서
> ⚠️ 각 단계는 **별도의 터미널에서 개별 실행**해야 합니다.

### **1️⃣ QGroundControl 실행**

> PX4 상태 모니터링 및 수동 이륙 제어용 GUI

---

### **2️⃣ XRCE Agent 실행 (PX4 ↔ ROS2 브릿지)**

```bash
source /opt/ros/humble/setup.bash
MicroXRCEAgent udp4 -p 8888
```

---

### **3️⃣ PX4 SITL 시뮬레이터 구동 (드론 스폰)**

```bash
source /opt/ros/humble/setup.bash
cd PX4-Autopilot
PX4_GZ_WORLD=aruco make px4_sitl gz_x500_lidar_down
```
> ✅ `aruco` 월드 내 `x500_lidar_down` 드론이 스폰됨

---

### **4️⃣ Gazebo ↔ ROS2 Bridge 실행, LiDAR PointCloud 브릿지 실행**

```bash
source /opt/ros/humble/setup.bash
source ~/rosgz/install/setup.bash
ros2 run ros_gz_bridge parameter_bridge /world/aruco/model/x500_lidar_down_0/link/lidar_sensor_link/sensor/lidar/scan/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked
```
> ✅ LiDAR 데이터를 Gazebo → ROS로 전달

---

### **6️⃣ Autoland Launch 실행 (자동 착륙 모듈)**

> 드론을 QGC에서 **수동 이륙 후 호버링 상태**로 만든 뒤 아래 실행

```bash
source /opt/ros/humble/setup.bash
source ~/px4_ros_ws/install/setup.bash
source ~/space_y/ws_KRAC/install/setup.bash
ros2 launch launch_package autoland.launch.py
```

> ✈️ 자동 착륙 노드가 동작하며,  
> Vision(`marker_recognition`) → Flight Control(`landing_test_vel`) → PX4 순으로 명령 전달됨

---

# 💡 참고 사항

- 각 터미널은 **각각(하나의 터미널이 아닌 총 6개!)으로 실행**되어야 함
- `autoland.launch.py` 안에서 **두 개의 노드**가 동시에 실행됨:
  - `marker_recognition` (Python 기반 비전 인식 노드)
  - `landing_test_vel` (C++ 기반 제어 노드)
- `/landing/coordinates` 토픽이 정상적으로 발행되는지 확인  
  ```bash
  ros2 topic echo /landing/coordinates
  ```

---

# ✅ 실행 순서 요약

| 순서 | 항목 | 명령어 요약 |
|:--:|:--|:--|
| 1 | QGroundControl 실행
| 2 | XRCE Agent 실행 | `MicroXRCEAgent udp4 -p 8888` |
| 3 | PX4 SITL 스폰 | `PX4_GZ_WORLD=aruco make px4_sitl gz_x500_lidar_down` |
| 4 | Gazebo ↔ ROS Bridge | `source ~/rosgz/install/setup.bash` |
| 5 | LiDAR 브릿지 | `ros2 run ros_gz_bridge parameter_bridge ...` |
| 6 | Autoland 실행 | `ros2 launch launch_package autoland.launch.py` |

---
