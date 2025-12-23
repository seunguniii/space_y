#!/usr/bin/env python3
from __future__ import annotations

import os
import time
import math
import struct
from typing import Optional, Tuple

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from px4_msgs.msg import VehicleOdometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2

def build_gst_pipeline() -> str:
    return (
            f"rtspsrc location={'rtsp://192.168.144.25:8554/main.264'} "
            "protocols=GST_RTSP_LOWER_TRANS_UDP "
            "latency=50 drop-on-latency=true do-retransmission=false ! "
            "rtph264depay ! h264parse ! "
            "avdec_h264 ! "
            "videoconvert ! video/x-raw,format=BGR ! "
            "appsink drop=1 max-buffers=1 sync=false"
        )


class MarkerRecognition(Node):
    _ARUCO_DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    try:
        _ARUCO_PARAMS = cv2.aruco.DetectorParameters()
    except AttributeError:  # OpenCV 버전 호환
        _ARUCO_PARAMS = cv2.aruco.DetectorParameters_create()

    _CAMERA_MATRIX = np.array(
        [[827.99145461, 0.0, 249.63373237],
         [0.0, 826.30893069, 260.11920342],
         [0.0, 0.0, 1.0]] ## -> 새로운 카메라 오면 꼭 반영하기
    )
    _DIST_COEFFS = np.array([[-0.27436478, 0.31753802, 0.00183457, -0.01212723, 0.05024013]])

    def __init__(self) -> None:
        super().__init__("marker_recognition")

        os.environ.setdefault("GST_DEBUG", "2")

        # ROS 파라미터 선언
        self.declare_parameter("frame_id", "camera_frame")
        self.declare_parameter("debug", True)
        self.declare_parameter("show_window", False)
        self.declare_parameter("use_filter", True)
        self.x_m = 0.
        self.y_m = 0.

        # 파라미터 값 읽기

        airframe_ = str(self.get_parameter("airframe").value)
        self._frame_id = str(self.get_parameter("frame_id").value)
        self._publish_debug = bool(self.get_parameter("debug").value)
        self._show_window = bool(self.get_parameter("show_window").value)

        self._filtered_z: Optional[float] = None
        mission_mode = "flight"
        self._altitude = 0.0
        src_param = build_gst_pipeline() #need further review


        # 카메라 열기
        self._cap = None
        self.get_logger().info(f"Trying to open as GStreamer pipeline:\n{src_param}")
        cap = cv2.VideoCapture(src_param, cv2.CAP_GSTREAMER)
        if cap.isOpened():
                self._cap = cap

        if self._cap is None or not self._cap.isOpened():
            self.get_logger().error("Unable to open camera")
            raise RuntimeError("Camera open failed")

        # 오도메트리 구독 (쿼터니언 -> roll/pitch)
        self._odom_sub = self.create_subscription(
            VehicleOdometry,
            "/fmu/out/vehicle_odometry",
            self._odom_cb,
            qos_profile=rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT, history=rclpy.qos.HistoryPolicy.KEEP_LAST, depth=10)
        )
        self._roll = 0.0
        self._pitch = 0.0


        # 퍼블리셔
        self._bridge = CvBridge()
        self._pub_point = self.create_publisher(PointStamped, "/landing/coordinates", 10)
        if self._publish_debug:
            from sensor_msgs.msg import Image  # Import here to avoid circular dependency if unused
            self._pub_img = self.create_publisher(Image, "/landing/video", 10)

        self._camera_timer = self.create_timer(1.0 / cam_rate, self._camera_timer_cb)

    # 오도메트리 콜백: 자세(roll,pitch) 계산
    def _odom_cb(self, msg: VehicleOdometry) -> None:
        #self.get_logger().info("Odomotery called")
        self._altitude = -msg.position[2]
        w, x, y, z = msg.q
        # Roll
        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        # Pitch
        sinp = 2.0 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2.0, sinp)
        else:
            pitch = math.asin(sinp)

        self._roll = roll
        self._pitch = pitch
        self._have_attitude = True


    # 카메라 프레임 처리
    def _camera_timer_cb(self) -> None:
        ret, frame = self._cap.read()
        if not ret:
            self.get_logger().error("Frame capture failed")
            return

        tag_centre = self._detect_first_tag(frame)
        if tag_centre is not None:
            cx, cy = tag_centre

            # ★ 픽셀 → 미터 변환 (카메라 내부 파라미터 사용)
            fx = self._CAMERA_MATRIX[0, 0]
            fy = self._CAMERA_MATRIX[1, 1]

            height, width = frame.shape[:2]
            cx0 = width/2
            cy0 = height/2

            dx = cx - cx0
            dy = cy0 - cy

            # self._latest_z는 보정된 카메라 높이(수직 z). 카메라 optical axis와 정렬 가정.
            self.x_m = dx/500
            self.y_m = dy/500


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
                    markerSize=10,
                    thickness=1,
                )

        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        msg.point.x = float(self.x_m)
        msg.point.y = float(self.y_m)
        msg.point.z = self._altitude
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
        # 왜곡 계수와 카메라 행렬 적용
        corners, ids, _ = cv2.aruco.detectMarkers(
            gray,
            self._ARUCO_DICT,
            parameters=self._ARUCO_PARAMS,cameraMatrix=self._CAMERA_MATRIX,
            distCoeff=self._DIST_COEFFS)
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
            if hasattr(node, "_cap"):
                node._cap.release()
        cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
