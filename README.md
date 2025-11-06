# space_y
Respository for flight control nodes and more. 


# 🚁 PX4-Gazebo Autoland System 실행 가이드

본 문서는 PX4 SITL 환경에서 **자동 착륙(autoland)** 기능을 실행하기 위한 전체 순서를 정리한 것입니다.  
각 단계는 **별도의 터미널**에서 실행해야 합니다.

---

## 🧭 실행 순서

### **1️⃣ QGroundControl 실행**
> PX4와의 통신 및 모니터링용 GUI

```bash
# QGroundControl 실행 (OS 환경에 따라 다를 수 있음)
./QGroundControl.AppImage
```

---

### **2️⃣ XRCE Agent 실행 (PX4 ↔ ROS2 통신 브릿지)**

```bash
source /opt/ros/humble/setup.bash
MicroXRCEAgent udp4 -p 8888
```

---

### **3️⃣ PX4 SITL 시뮬레이터 구동 (드론 스폰)**

```bash
source /opt/ros/humble/setup.bash
PX4_GZ_WORLD=aruco make px4_sitl gz_x500_lidar_down
```

> ✅ `aruco` 월드에서 `x500_lidar_down` 모델이 스폰됨

---

### **4️⃣ Gazebo ↔ ROS2 Bridge 실행 (기본 bridge)**

```bash
source /opt/ros/humble/setup.bash
source ~/rosgz/install/setup.bash
```

> 이후 Gazebo에서 ROS2로 기본 토픽 브릿징 수행

---

### **5️⃣ LiDAR PointCloud 브릿지 실행**

```bash
source /opt/ros/humble/setup.bash
ros2 run ros_gz_bridge parameter_bridge /world/aruco/model/x500_lidar_down_0/link/lidar_sensor_link/sensor/lidar/scan/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked
```

> ✅ Gazebo LiDAR 데이터를 ROS2 `PointCloud2` 메시지로 변환

---

### **6️⃣ Autoland Launch 실행**

드론이 **이륙 후 호버링 상태**에 있을 때 실행합니다.

```bash
source /opt/ros/humble/setup.bash
source ~/ws_sensor_combined/install/setup.bash
source ~/space_y/ws_KRAC/install/setup.bash
ros2 launch launch_package autoland.launch.py
```

> ✈️ 자동 착륙 노드가 실행되며,  
> Vision 노드(`marker_recognition`) → Flight Control(`landing_test_vel`) → PX4 순으로 동작합니다.

---

## 💡 참고 사항

- 각 터미널은 **별도로 실행**해야 함 (병렬 구동 필수)
- `autoland.launch.py` 내에서 두 개의 노드가 동시에 실행됨:
  - `marker_recognition` (Python)
  - `landing_test_vel` (C++)
- 착륙 시, `/landing/coordinates` 토픽이 정상적으로 발행되는지 확인  
  (`ros2 topic echo /landing/coordinates`)

---

## ✅ 실행 순서 요약

| 순서 | 항목 | 명령어 요약 |
|:--:|:--|:--|
| 1 | QGroundControl 실행 | `./QGroundControl.AppImage` |
| 2 | XRCE Agent | `MicroXRCEAgent udp4 -p 8888` |
| 3 | PX4 SITL 스폰 | `PX4_GZ_WORLD=aruco make px4_sitl gz_x500_lidar_down` |
| 4 | Gazebo ↔ ROS Bridge | `source ~/rosgz/install/setup.bash` |
| 5 | LiDAR 브릿지 | `ros2 run ros_gz_bridge parameter_bridge ...` |
| 6 | Autoland 실행 | `ros2 launch launch_package autoland.launch.py` |

---
