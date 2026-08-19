"""ROS 2 synchronized RGB-D dataset recorder for segmentation training."""

import json
from pathlib import Path

import cv2
import message_filters
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

from .node import _image_array


class DatasetRecorder(Node):
    def __init__(self):
        super().__init__("hexapod_dataset_recorder")
        defaults = {
            "output_dir": "dataset_capture",
            "color_topic": "/camera/color/image_raw",
            "depth_topic": "/camera/depth/image_raw",
            "camera_info_topic": "/camera/color/camera_info",
            "sample_period": 0.5,
            "max_samples": 500,
            "save_depth": True,
            "minimum_valid_depth_ratio": 0.10,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        p = lambda name: self.get_parameter(name).value
        self.output = Path(p("output_dir")).expanduser().resolve()
        self.images_dir = self.output / "images_raw"
        self.depth_dir = self.output / "depth_m"
        self.images_dir.mkdir(parents=True, exist_ok=True)
        if p("save_depth"):
            self.depth_dir.mkdir(parents=True, exist_ok=True)
        self.sample_period = float(p("sample_period"))
        self.max_samples = int(p("max_samples"))
        self.save_depth = bool(p("save_depth"))
        self.minimum_valid_ratio = float(p("minimum_valid_depth_ratio"))
        self.last_saved_ns = None
        self.saved = 0
        self.camera_info = None
        self.info_sub = self.create_subscription(
            CameraInfo, p("camera_info_topic"), self._info, qos_profile_sensor_data)
        color = message_filters.Subscriber(
            self, Image, p("color_topic"), qos_profile=qos_profile_sensor_data)
        depth = message_filters.Subscriber(
            self, Image, p("depth_topic"), qos_profile=qos_profile_sensor_data)
        sync = message_filters.ApproximateTimeSynchronizer([color, depth], 30, 0.04)
        sync.registerCallback(self._frame)
        self._sync_handles = (color, depth, sync)
        self.get_logger().info(
            f"recording to {self.output}; period={self.sample_period:.2f}s, "
            f"maximum={self.max_samples}")

    def _info(self, message):
        self.camera_info = message

    def _frame(self, color_message, depth_message):
        if self.camera_info is None or self.saved >= self.max_samples:
            return
        timestamp_ns = (color_message.header.stamp.sec * 1_000_000_000
                        + color_message.header.stamp.nanosec)
        if self.last_saved_ns is not None:
            if (timestamp_ns - self.last_saved_ns) / 1e9 < self.sample_period:
                return
        rgb = _image_array(color_message)
        raw = _image_array(depth_message)
        if rgb.shape[:2] != raw.shape:
            self.get_logger().error("RGB/depth size mismatch; enable D2C registration")
            return
        depth = raw.astype(np.float32)
        if depth_message.encoding == "16UC1":
            depth *= 0.001
            depth[raw == np.iinfo(np.uint16).max] = np.nan
        depth[(depth <= 0) | ~np.isfinite(depth)] = np.nan
        valid_ratio = float(np.isfinite(depth).mean())
        if valid_ratio < self.minimum_valid_ratio:
            self.get_logger().warning(
                f"skip low-depth frame: valid={valid_ratio:.1%}",
                throttle_duration_sec=2.0)
            return
        stem = f"frame_{timestamp_ns}"
        image_path = self.images_dir / f"{stem}.png"
        if not cv2.imwrite(str(image_path), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)):
            self.get_logger().error(f"failed to write {image_path}")
            return
        if self.save_depth:
            np.save(self.depth_dir / f"{stem}.npy", depth)
        depth_ns = (depth_message.header.stamp.sec * 1_000_000_000
                    + depth_message.header.stamp.nanosec)
        metadata = {
            "timestamp_ns": timestamp_ns,
            "color_frame_id": color_message.header.frame_id,
            "depth_frame_id": depth_message.header.frame_id,
            "color_depth_dt_ms": abs(timestamp_ns - depth_ns) / 1e6,
            "width": color_message.width,
            "height": color_message.height,
            "depth_encoding": depth_message.encoding,
            "depth_unit": "m",
            "valid_depth_ratio": valid_ratio,
            "K": list(self.camera_info.k),
            "D": list(self.camera_info.d),
        }
        with (self.output / "metadata.jsonl").open("a", encoding="utf-8") as stream:
            stream.write(json.dumps(metadata, ensure_ascii=False) + "\n")
        self.last_saved_ns = timestamp_ns
        self.saved += 1
        self.get_logger().info(f"saved {self.saved}/{self.max_samples}: {stem}")
        if self.saved == self.max_samples:
            self.get_logger().info("maximum samples reached; recording stopped")


def main(args=None):
    rclpy.init(args=args)
    node = DatasetRecorder()
    try:
        while rclpy.ok() and node.saved < node.max_samples:
            rclpy.spin_once(node, timeout_sec=1.0)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
