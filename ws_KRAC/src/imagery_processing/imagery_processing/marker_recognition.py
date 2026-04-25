#!/usr/bin/env python3
from __future__ import annotations

import os
import math
import struct
from tracemalloc import start
from typing import Optional, Tuple

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from px4_msgs.msg import VehicleCommand, DistanceSensor
from smbus import SMBus
import time
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

def build_gst_pipeline() -> str:
    return (
        "rtspsrc location=rtsp://192.168.0.20:8554/main.264 "
        "protocols=GST_RTSP_LOWER_TRANS_UDP "
        "latency=50 drop-on-latency=true do-retransmission=false ! "
        "rtph265depay ! h265parse ! "
        "avdec_h265 ! "
        "videoconvert ! video/x-raw,format=BGR ! "
        "appsink drop=1 max-buffers=1 sync=false"
    )


class MarkerRecognition(Node):
    
    # 라이다 상수
    I2C_BUS = 7
    LIDAR_ADDR = 0x62

    ACQ_COMMAND = 0x00
    STATUS = 0x01
    DISTANCE_HIGH = 0x0F
    DISTANCE_LOW = 0x10

    _ARUCO_DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    try:
        _ARUCO_PARAMS = cv2.aruco.DetectorParameters()
    except AttributeError:
        _ARUCO_PARAMS = cv2.aruco.DetectorParameters_create()

    _CAMERA_MATRIX = np.array(
        [           
            [831.47721015,0.0,639.50000022],
            [0.0,74.00176077,359.50000105],
            [0.0,0.0,1.0]

        ]
    )
    _DIST_COEFFS = np.array(
        [-9.11342523e-03, 2.09416645e-05, 2.45889764e-03, 3.30179975e-05, -1.30731372e-08]
    )

    def __init__(self) -> None:
        super().__init__("marker_recognition")

        os.environ.setdefault("GST_DEBUG", "2")

        # ROS parameters
        self.declare_parameter("frame_id", "camera_frame")
        self.declare_parameter("debug", True)
        self.declare_parameter("show_window", False)
        self.declare_parameter("use_filter", True)
        self.declare_parameter("frame_rate", 30.0)
        self.declare_parameter("lidar_altitude", 0.12)

        # Parameter values
        self._frame_id = str(self.get_parameter("frame_id").value)
        self._publish_debug = bool(self.get_parameter("debug").value)
        self._show_window = bool(self.get_parameter("show_window").value)
        self.frame_rate = float(self.get_parameter("frame_rate").value)
        self._lidar_altitude = float(self.get_parameter("lidar_altitude").value)
        
        self._i2c_bus = SMBus(7)
        self._lidar_timer = self.create_timer(0.05, self._lidar_timer_cb)  # 20 Hz
        
        # Internal states
        self.x_m = 0.0
        self.y_m = 0.0
        self._filtered_z: Optional[float] = None
        self._altitude = 0.0
        self._roll = 0.0
        self._pitch = 0.0
        self._have_attitude = False
        self._mission_mode = "flight"

        # Camera open
        src_param = build_gst_pipeline()
        self._cap = None
        self.get_logger().info(f"Trying to open as GStreamer pipeline:\n{src_param}")
        cap = cv2.VideoCapture(src_param, cv2.CAP_GSTREAMER)
        if cap.isOpened():
            self._cap = cap

        if self._cap is None or not self._cap.isOpened():
            self.get_logger().error("Unable to open camera")
            raise RuntimeError("Camera open failed")

        # Gimbal publisher: VehicleCommand 방식
        self._gimbal_pub = self.create_publisher(
            VehicleCommand,
            "/fmu/in/vehicle_command",
            10,
        )
        distance_qos = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=10,)

        self._distance_sensor_pub = self.create_publisher(DistanceSensor,"/fmu/in/distance_sensor",distance_qos,)
        self._gimbal_configured = False
        
        self._mission_sub = self.create_subscription(
            String,
            "mission_mode",
            self._mission_cb,
            10,
        )

        # Publishers
        self._bridge = CvBridge()
        self._pub_point = self.create_publisher(PointStamped, "/landing/coordinates", 10)

        if self._publish_debug:
            from sensor_msgs.msg import Image
            self._pub_img = self.create_publisher(Image, "/landing/video", 10)

        # Timers
        self._gimbal_timer = self.create_timer(1.0, self._set_gimbal_pitch_down)
        self._camera_timer = self.create_timer(1.0 / self.frame_rate, self._camera_timer_cb)

    def _mission_cb(self, msg: String) -> None:
        self._mission_mode = msg.data
        

    
    def _wait_lidar_ready(self, timeout: float = 0.05) -> bool:
        start = time.time()

        while time.time() - start < timeout:
            status = self._i2c_bus.read_byte_data(self.LIDAR_ADDR, self.STATUS)

        # status bit 0 == busy
            if (status & 0x01) == 0:
                return True

            time.sleep(0.001)

        return False


    def _read_lidar_distance_cm(self) -> int:
        # Start measurement
        self._i2c_bus.write_byte_data(
            self.LIDAR_ADDR,
            self.ACQ_COMMAND,
            0x04
        )

        if not self._wait_lidar_ready():
            raise TimeoutError("Lidar Lite v3HP measurement timeout")

        high = self._i2c_bus.read_byte_data(
            self.LIDAR_ADDR,
            self.DISTANCE_HIGH
        )

        low = self._i2c_bus.read_byte_data(
            self.LIDAR_ADDR,
            self.DISTANCE_LOW
        )

        distance_cm = (high << 8) | low
        return distance_cm


    def _lidar_timer_cb(self) -> None:
        try:
            distance_cm = self._read_lidar_distance_cm()
            distance_m = distance_cm / 100.0

            # 카메라/기체 기준 offset 보정값
            self._altitude = distance_m - self._lidar_altitude

            if not math.isfinite(self._altitude):
                return

            # PX4 DistanceSensor publish
            msg = DistanceSensor()
            msg.timestamp = self.get_clock().now().nanoseconds // 1000  # us

            # 임의 device id. 센서 하나면 0으로 둬도 됨
            msg.device_id = 0

            msg.min_distance = 0.05
            msg.max_distance = 40.0
            msg.current_distance = float(distance_m)

            # 정확한 분산값 모르면 0.0
            msg.variance = 0.0

            # MAV_DISTANCE_SENSOR_LASER = 0
            msg.type = 0

            # FOV 모르면 0.0
            msg.h_fov = 0.0
            msg.v_fov = 0.0

            # PX4 distance sensor orientation
            # 보통 downward-facing = 25
            msg.orientation = 25

            # orientation enum을 쓰는 경우 q는 NaN으로 둠
            try:
                msg.q = [float("nan"), float("nan"), float("nan"), float("nan")]
            except Exception:
                pass

            # px4_msgs 버전에 따라 있을 수도/없을 수도 있음
            try:
                msg.signal_quality = 100
            except Exception:
                pass

            self._distance_sensor_pub.publish(msg)

            self.get_logger().info(
                f"lidar altitude: {self._altitude:.3f} m"
            )

        except Exception as e:
            self.get_logger().warn(f"lidar read/publish failed: {e}")

    
    def _publish_vehicle_command(
        self,
        command: int,
        param1: float = 0.0,
        param2: float = 0.0,
        param3: float = 0.0,
        param4: float = 0.0,
        param5: float = 0.0,
        param6: float = 0.0,
        param7: float = 0.0,
    ) -> None:
        msg = VehicleCommand()
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        msg.param1 = param1
        msg.param2 = param2
        msg.param3 = param3
        msg.param4 = param4
        msg.param5 = param5
        msg.param6 = param6
        msg.param7 = param7
        msg.command = command

        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 191
        msg.from_external = True

        self._gimbal_pub.publish(msg)

    def _set_gimbal_pitch_down(self) -> None:
        # 1) 처음 한 번은 gimbal manager control ownership 요청
        if not self._gimbal_configured:
            self._publish_vehicle_command(
                command=VehicleCommand.VEHICLE_CMD_DO_GIMBAL_MANAGER_CONFIGURE,
                param1=1.0,      # primary control sysid
                param2=191.0,    # primary control compid
                param3=-1.0,     # secondary sysid unused
                param4=-1.0,     # secondary compid unused
                param5=0.0,
                param6=0.0,
                param7=154.0,    # gimbal device id
            )
            self._gimbal_configured = True
            self.get_logger().info("Sent gimbal configure command")
            return

        # 2) 이후 pitch/yaw command 전송
        self._publish_vehicle_command(
            command=VehicleCommand.VEHICLE_CMD_DO_GIMBAL_MANAGER_PITCHYAW,
            param1=-90.0,   # pitch [deg]
            param2=0.0,     # yaw [deg]
            param3=0.0,     # pitch rate [deg/s]
            param4=0.0,     # yaw rate [deg/s]
            param5=0.0,     # flags (body frame)
            param6=0.0,
            param7=154.0,   # gimbal device id
        )

    def _camera_timer_cb(self) -> None:
        ret, frame = self._cap.read()
        if not ret:
            self.get_logger().error("Frame capture failed")
            return

        tag_centre = self._detect_first_tag(frame)

        if tag_centre is not None:
            cx, cy = tag_centre

            fx = self._CAMERA_MATRIX[0, 0]
            fy = self._CAMERA_MATRIX[1, 1]

            height, width = frame.shape[:2]
            cx0 = width / 2.0
            cy0 = height / 2.0

            dx = cx - cx0
            dy = cy0 - cy

            z = self._altitude
            if not np.isfinite(z) or z < 0.05:
                z = float("nan")

            self.x_m = dx / fx * z
            self.y_m = dy / fy * z

            if self._publish_debug:
                cv2.drawMarker(
                    frame,
                    (int(cx), int(cy)),
                    (0, 255, 0),
                    markerType=cv2.MARKER_CROSS,
                    markerSize=20,
                    thickness=2,
                )
                cv2.drawMarker(
                    frame,
                    (int(cx0), int(cy0)),
                    (255, 0, 0),
                    markerType=cv2.MARKER_CROSS,
                    markerSize=20,
                    thickness=2,
                )
        else:
            self.x_m = float("nan")
            self.y_m = float("nan")

        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        msg.point.x = float(self.x_m)
        msg.point.y = float(self.y_m)
        msg.point.z = float(self._altitude)
        self._pub_point.publish(msg)

        if self._publish_debug:
            self._publish_image(frame)

    def _publish_image(self, frame: np.ndarray) -> None:
        img_msg = self._bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        img_msg.header.stamp = self.get_clock().now().to_msg()
        img_msg.header.frame_id = self._frame_id
        self._pub_img.publish(img_msg)

    def _detect_first_tag(self, frame: np.ndarray) -> Optional[Tuple[float, float]]:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(
            gray,
            self._ARUCO_DICT,
            parameters=self._ARUCO_PARAMS,
            cameraMatrix=self._CAMERA_MATRIX,
            distCoeff=self._DIST_COEFFS,
        )

        if ids is None or len(ids) == 0:
            return None

        pts = corners[0].reshape(4, 2)
        cx = float(np.mean(pts[:, 0]))
        cy = float(np.mean(pts[:, 1]))
        return cx, cy


def main(args=None):
    rclpy.init(args=args)
    node: Optional[MarkerRecognition] = None

    try:
        node = MarkerRecognition()
        rclpy.spin(node)
    except Exception:
        import traceback
        traceback.print_exc()
    finally:
        if node is not None:
            node.destroy_node()
            if hasattr(node, "_cap") and node._cap is not None:
                node._cap.release()

        cv2.destroyAllWindows()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()