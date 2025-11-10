# Raspberry Pi 4 설정 방법

본 문서는 Raspberry Pi 4를 PX4의 Compainon Computer로 활용하기 위해 설정하는 방법을 설명합니다.  
ROS2 Humble + uORB환경 기준입니다.

---

# 🧩 설치 순서

1. [**Ubuntu 22.04 설치**](https://ubuntu.com/tutorials/how-to-install-ubuntu-desktop-on-raspberry-pi-4#2-prepare-the-sd-card)
2. [**ROS2 Humble 설치**](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)  
   ros-humble-ros-base를 설치하도록 한다. 필요에 따라서 ros-humble-dev-tools 등도 설치한다.
3. [**Raspberry Pi 환경 설정**](https://docs.px4.io/main/en/companion_computer/pixhawk_rpi#ubuntu-setup-on-rpi)
4. [**Micro XRCE-DDS Agent 설치**](https://docs.px4.io/main/en/companion_computer/pixhawk_rpi#ros-setup-on-rpi)
5. **OpenCV 4.5.4에 GStreamer 설치 여부 확인 (중요)**  
   ```python
   import cv2
   print(cv2.getBuildInformation())
   ```
   ✅ `GStreamer: YES` 확인 필수
6. **SSH 설치 및 실행**
   설치하기:
   ```bash
   sudo apt update && sudo apt upgrade
   sudo apt-get install openssh-server
   sudo apt-get install ssh
   ```
   실행하기:
   ```bash
   sudo service ssh start
   ```
7. **지상국에 Putty 설치 및 실행**
