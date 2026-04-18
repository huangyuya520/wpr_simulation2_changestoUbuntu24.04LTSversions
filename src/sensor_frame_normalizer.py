#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import Image, Imu, LaserScan, PointCloud2


def normalize_frame_id(frame_id: str) -> str:
    normalized = frame_id.lstrip("/").replace("::", "/")
    tokens = [token for token in normalized.split("/") if token]
    if not tokens:
        return normalized
    return tokens[-1]


class SensorFrameNormalizer(Node):
    def __init__(self) -> None:
        super().__init__("sensor_frame_normalizer")

        self._reported_pairs: set[tuple[str, str]] = set()
        scan_pub_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.scan_pub = self.create_publisher(LaserScan, "/scan", scan_pub_qos)
        self.imu_pub = self.create_publisher(Imu, "/imu/data", qos_profile_sensor_data)
        self.image_pub = self.create_publisher(
            Image, "/kinect2/qhd/image_raw", qos_profile_sensor_data
        )
        self.points_pub = self.create_publisher(
            PointCloud2, "/kinect2/sd/points", qos_profile_sensor_data
        )

        self.create_subscription(
            LaserScan,
            "/scan_bridge",
            lambda msg: self._normalize_and_publish(msg, self.scan_pub, "scan"),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Imu,
            "/imu/data_bridge",
            lambda msg: self._normalize_and_publish(msg, self.imu_pub, "imu"),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            "/kinect2/qhd/image_raw_bridge",
            lambda msg: self._normalize_and_publish(msg, self.image_pub, "image"),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PointCloud2,
            "/kinect2/sd/points_bridge",
            lambda msg: self._normalize_and_publish(msg, self.points_pub, "pointcloud"),
            qos_profile_sensor_data,
        )

    def _normalize_and_publish(self, msg, publisher, label: str) -> None:
        raw_frame_id = msg.header.frame_id
        normalized_frame_id = normalize_frame_id(raw_frame_id)
        if normalized_frame_id:
            msg.header.frame_id = normalized_frame_id

        pair = (raw_frame_id, msg.header.frame_id)
        if raw_frame_id != msg.header.frame_id and pair not in self._reported_pairs:
            # Why: Gazebo may prepend scoped model paths to frame ids, but SLAM/TF use URDF leaf frame names.
            self.get_logger().info(
                f"Normalized {label} frame_id from '{raw_frame_id}' to '{msg.header.frame_id}'."
            )
            self._reported_pairs.add(pair)

        publisher.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SensorFrameNormalizer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
