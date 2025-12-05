#!/usr/bin/env python3
"""Bridge inspection events to torch toggle commands."""

from __future__ import annotations

from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String

from ..topics import namespaced


class TorchController(Node):
    """Forwards inspection events to the torch command topic."""

    def __init__(self) -> None:
        super().__init__("torch_controller")
        self._vehicle_ns = (
            self.declare_parameter("vehicle_ns", "").get_parameter_value().string_value.strip()
        )
        self.declare_parameter("command_topic", "overrack/torch_enable")
        self.declare_parameter("startup_on", False)
        self.declare_parameter("min_on_seconds", 10.0)

        self._command_topic = self._resolve_topic(
            self.get_parameter("command_topic").get_parameter_value().string_value
        )
        self._inspection_topic = namespaced("overrack/inspection", namespace=self._vehicle_ns)
        self._enabled = False
        self._last_on_ts: Optional[float] = None
        self._min_on = max(0.0, float(self.get_parameter("min_on_seconds").value))
        self._cmd_pub = self.create_publisher(Bool, self._command_topic, 10)
        self.create_subscription(String, self._inspection_topic, self._on_inspection_event, 10)
        self.get_logger().info(
            "Torch bridge ready (cmd_topic=%s, inspection_topic=%s, startup_on=%s, min_on_s=%.1f)"
            % (
                self._command_topic,
                self._inspection_topic,
                self.get_parameter("startup_on").value,
                self._min_on,
            )
        )

        if bool(self.get_parameter("startup_on").value):
            self._last_on_ts = self._now()
            self._publish(True, log=True)

    def _on_inspection_event(self, msg: String) -> None:
        payload = (msg.data or "").strip().upper()
        now = self._now()
        if payload == "LOW_LIGHT":
            self._last_on_ts = now
            self._publish(True, log=True)
        elif payload == "OK":
            if self._enabled and self._min_on > 0.0 and self._last_on_ts is not None:
                elapsed = now - self._last_on_ts
                if elapsed < self._min_on:
                    remaining = self._min_on - elapsed
                    self.get_logger().debug(
                        "Torch hold: %.1fs remaining before OFF (min_on_s=%.1f)"
                        % (remaining, self._min_on)
                    )
                    return
            self._publish(False, log=True)

    def _publish(self, enabled: bool, *, log: bool = False) -> None:
        if enabled == self._enabled:
            return
        self._enabled = enabled
        out = Bool()
        out.data = enabled
        self._cmd_pub.publish(out)
        if log:
            state = "ON" if enabled else "OFF"
            self.get_logger().info(f"Torch {state}")

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _resolve_topic(self, topic: str) -> str:
        if not self._vehicle_ns:
            return topic
        if topic.startswith(f"/{self._vehicle_ns}/") or topic.startswith(f"{self._vehicle_ns}/"):
            return topic
        return namespaced(topic, namespace=self._vehicle_ns)


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node: Optional[TorchController] = None
    try:
        node = TorchController()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
