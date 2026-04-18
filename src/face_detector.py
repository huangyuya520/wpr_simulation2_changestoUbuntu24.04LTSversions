#!/usr/bin/env python3

from pathlib import Path
import rclpy
import sys
import cv2
from rclpy.node import Node
from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from sensor_msgs.msg import RegionOfInterest


def resolve_haar_cascade_path() -> str:
    candidates = [
        Path("/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml"),
        Path("/usr/share/opencv/haarcascades/haarcascade_frontalface_default.xml"),
        Path("/usr/local/share/opencv4/haarcascades/haarcascade_frontalface_default.xml"),
    ]

    for candidate in candidates:
        if candidate.is_file():
            return str(candidate)

    raise FileNotFoundError(
        "Could not find haarcascade_frontalface_default.xml in the standard OpenCV locations."
    )


class FaceDetector(Node):
    def __init__(self):
        super().__init__("face_detector")
        self.pub = self.create_publisher(RegionOfInterest, "/face_position", 10)
        self.sub = self.create_subscription(
            Image, "/face_detector_input", self.detect_faces_callback, 10
        )
        self.bridge = CvBridge()
        self.face_detector = cv2.CascadeClassifier(resolve_haar_cascade_path())
        if self.face_detector.empty():
            raise RuntimeError("Failed to load OpenCV frontal-face Haar cascade.")
        self.frame_count = 0
        self.face_count = 0
        self.get_logger().info("OpenCV Haar face detector is ready.")

    def detect_faces_callback(self, data):
        frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray_frame = cv2.equalizeHist(gray_frame)

        faces = self.face_detector.detectMultiScale(
            gray_frame,
            scaleFactor=1.1,
            minNeighbors=5,
            minSize=(30, 30),
        )

        self.frame_count += 1
        if self.frame_count == 1:
            self.get_logger().info(
                f"Received first detector frame: {frame.shape[1]}x{frame.shape[0]}"
            )

        if len(faces) == 0:
            self.get_logger().warn("当前帧未检测到人脸。")
            return

        self.face_count += len(faces)
        self.get_logger().info(f"当前帧检测到 {len(faces)} 张人脸。")

        for (x, y, w, h) in faces:
            roi_msg = RegionOfInterest()
            roi_msg.x_offset = int(x)
            roi_msg.y_offset = int(y)
            roi_msg.width = int(w)
            roi_msg.height = int(h)
            self.pub.publish(roi_msg)


def main(args):
    rclpy.init(args=args)
    face_detector_object = FaceDetector()
    face_detector_object.get_logger().warn("准备识别人脸")

    try:
        rclpy.spin(face_detector_object)
    except KeyboardInterrupt as e:
        print(e)

    face_detector_object.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main(sys.argv)
