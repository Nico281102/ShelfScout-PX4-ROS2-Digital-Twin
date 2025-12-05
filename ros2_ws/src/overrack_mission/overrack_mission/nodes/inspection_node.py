"""Simple inspection node emitting OK/SUSPECT/LOW_LIGHT events from camera frames."""

from __future__ import annotations

from pathlib import Path
from typing import Optional

import rclpy
from rclpy.node import Node
try:
    from rclpy.qos import SensorDataQoS  # Humble extras
except ImportError:  # pragma: no cover - fallback for distros without helper
    from rclpy.qos import QoSPresetProfiles

    def SensorDataQoS():
        return QoSPresetProfiles.get_from_short_key("sensor_data")
from std_msgs.msg import String
from sensor_msgs.msg import Image

try:  # pragma: no cover - numpy is optional at runtime
    import numpy as np
except Exception:  # noqa: BLE001
    np = None

from ..px4io.qos import EVENTS_QOS
from ..px4io.topics import namespaced


class InspectionNode(Node):
    """Basic inspection pipeline using luminance heuristics.

    The node is intentionally simple: it estimates image brightness/contrast and emits:
    * LOW_LIGHT if average luminance < low_light_threshold
    * SUSPECT if contrast is below suspect_variance threshold (poor texture / unreadable barcode)
    * OK otherwise
    """

    def __init__(self) -> None:
        super().__init__("inspection_node")
        self._vehicle_ns = (
            self.declare_parameter("vehicle_ns", "").get_parameter_value().string_value.strip()
        )
        self.declare_parameter("image_topic", "/overrack/iris/front_camera/image_raw")
        self.declare_parameter("low_light_threshold", 40.0)
        self.declare_parameter("suspect_variance", 12.0)
        self.declare_parameter("publish_period_s", 0.5)

        image_topic = self.get_parameter("image_topic").get_parameter_value().string_value
        self._image_topic = self._resolve_topic(image_topic)
        self._low_light_threshold = float(self.get_parameter("low_light_threshold").value)
        self._suspect_variance = float(self.get_parameter("suspect_variance").value)
        self._publish_period = max(0.1, float(self.get_parameter("publish_period_s").value))

        self._event_pub = self.create_publisher(
            String, namespaced("overrack/inspection", namespace=self._vehicle_ns), EVENTS_QOS
        )
        self._image_sub = self.create_subscription(
            Image,
            self._image_topic,
            self._on_image,
            SensorDataQoS(),
        )
        self._state_sub = self.create_subscription(
            String,
            namespaced("overrack/mission_state", namespace=self._vehicle_ns),
            self._on_mission_state,
            EVENTS_QOS,
        )

        self.get_logger().info(
            "Inspection node active (image_topic=%s, publish_period=%.2fs)"
            % (self._image_topic, self._publish_period)
        )

        self._last_event_stamp: Optional[float] = None
        self._pending_snapshots = 0
        self._last_image: Optional[Image] = None
        if self._vehicle_ns:
            self._snapshot_dir = Path(f"data/images/{self._vehicle_ns}/snapshots")
        else:
            self._snapshot_dir = Path("data/images/snapshots")
        self._snapshot_dir.mkdir(parents=True, exist_ok=True)
        self._snapshot_index = 0

    # ------------------------------------------------------------------
    # Callbacks

    def _on_image(self, msg: Image) -> None:
        self._last_image = msg
        now = self.get_clock().now().nanoseconds / 1e9
        force = False
        if self._pending_snapshots > 0 and self._maybe_handle_snapshot(msg):
            force = True
        elif self._last_event_stamp is not None and (now - self._last_event_stamp) < self._publish_period:
            return

        luminance, variance = self._compute_luminance(msg)
        if luminance is None:
            return

        if luminance < self._low_light_threshold:
            self._publish("LOW_LIGHT")
        elif variance < self._suspect_variance:
            self._publish("SUSPECT")
        else:
            self._publish("OK")
        if force or self._publish_period > 0.0:
            self._last_event_stamp = now

    def _on_mission_state(self, msg: String) -> None:
        payload = (msg.data or "").strip().lower()
        if payload == "snapshot":
            self._pending_snapshots += 1
            if self._last_image is None:
                self.get_logger().debug("Snapshot requested but no image received yet")

    # ------------------------------------------------------------------
    # Helpers

    def _publish(self, label: str) -> None:
        msg = String()
        msg.data = label
        self._event_pub.publish(msg)

    def _compute_luminance(self, msg: Image) -> tuple[Optional[float], Optional[float]]:
        if msg.encoding not in {"mono8", "rgb8", "bgr8"}:
            self.get_logger().warn_once(f"Unsupported image encoding: {msg.encoding}")
            return (None, None)
        if msg.encoding == "mono8":
            luminances = memoryview(msg.data)
            if not luminances:
                return (None, None)
            return self._stats_from_buffer(luminances)
        # rgb/bgr
        step = 3
        raw = memoryview(msg.data)
        if len(raw) < step:
            return (None, None)
        if np is not None:
            array = np.frombuffer(raw, dtype=np.uint8)
            array = array.reshape((-1, step))
            luminance = array.mean(axis=1)
            return float(luminance.mean()), float(luminance.var())
        # Fallback without numpy
        luminances = []
        for i in range(0, len(raw), step):
            try:
                r, g, b = raw[i], raw[i + 1], raw[i + 2]
            except IndexError:
                break
            if msg.encoding == "rgb8":
                luminances.append(0.2126 * r + 0.7152 * g + 0.0722 * b)
            else:  # bgr8
                luminances.append(0.2126 * b + 0.7152 * g + 0.0722 * r)
        return self._stats_from_buffer(luminances)

    def _stats_from_buffer(self, data) -> tuple[Optional[float], Optional[float]]:
        if np is not None and hasattr(data, "__array__"):
            arr = np.asarray(data, dtype=np.float32)
            return float(arr.mean()), float(arr.var())
        data = list(data)
        if not data:
            return (None, None)
        mean = sum(data) / len(data)
        variance = sum((x - mean) ** 2 for x in data) / len(data)
        return (mean, variance)

    def _maybe_handle_snapshot(self, image: Image) -> bool:
        if self._pending_snapshots <= 0:
            return False
        if self._write_snapshot(image):
            self._pending_snapshots -= 1
            return True
        return False

    def _write_snapshot(self, image: Image) -> bool:
        encoding = (image.encoding or "").lower()
        try:
            if encoding == "mono8":
                payload = self._copy_rows(image, channels=1)
                header = f"P5\n{image.width} {image.height}\n255\n".encode("ascii")
                extension = "pgm"
            elif encoding in {"rgb8", "bgr8"}:
                payload = self._copy_rows(image, channels=3)
                if encoding == "bgr8":
                    payload = self._bgr_to_rgb(payload)
                header = f"P6\n{image.width} {image.height}\n255\n".encode("ascii")
                extension = "ppm"
            else:
                self.get_logger().warn_once(f"Snapshot skipped: unsupported encoding '{image.encoding}'")
                return False
        except ValueError as exc:
            self.get_logger().error(f"Snapshot failed: {exc}")
            return False

        self._snapshot_index += 1
        ts_ms = int(self.get_clock().now().nanoseconds / 1e6)
        filename = self._snapshot_dir / f"snapshot_{self._snapshot_index:04d}_{ts_ms}.{extension}"
        try:
            with filename.open("wb") as handle:
                handle.write(header)
                handle.write(payload)
        except OSError as exc:
            self.get_logger().error(f"Failed to write snapshot {filename}: {exc}")
            return False

        self.get_logger().info(f"Snapshot saved to {filename}")
        return True

    def _copy_rows(self, image: Image, *, channels: int) -> bytes:
        row_width = image.width * channels
        step = image.step
        if step < row_width:
            raise ValueError(f"Image step ({step}) smaller than expected row width ({row_width})")
        raw = memoryview(image.data)
        if len(raw) < step * image.height:
            raise ValueError("Image data buffer smaller than declared dimensions")
        buffer = bytearray(row_width * image.height)
        for row in range(image.height):
            src = row * step
            dst = row * row_width
            buffer[dst : dst + row_width] = raw[src : src + row_width]
        return bytes(buffer)

    @staticmethod
    def _bgr_to_rgb(data: bytes) -> bytes:
        buffer = bytearray(len(data))
        for idx in range(0, len(data), 3):
            try:
                b, g, r = data[idx], data[idx + 1], data[idx + 2]
            except IndexError:
                break
            buffer[idx] = r
            buffer[idx + 1] = g
            buffer[idx + 2] = b
        return bytes(buffer)

    def _resolve_topic(self, topic: str) -> str:
        """Apply namespace unless topic already matches the namespace."""
        if not self._vehicle_ns:
            return topic
        clean_topic = topic or ""
        if clean_topic.startswith(f"/{self._vehicle_ns}/") or clean_topic.startswith(f"{self._vehicle_ns}/"):
            return clean_topic
        return namespaced(clean_topic, namespace=self._vehicle_ns)


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node: Optional[InspectionNode] = None
    try:
        node = InspectionNode()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":  # pragma: no cover
    main()
