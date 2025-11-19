#!/usr/bin/env python3
"""Bridge inspection events to torch toggle commands."""

from __future__ import annotations

from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String


class TorchController(Node):
    """Forwards inspection events to the torch command topic."""

    _INSPECTION_TOPIC = "overrack/inspection"

    def __init__(self) -> None:
        super().__init__("torch_controller")
        self.declare_parameter("command_topic", "overrack/torch_enable")
        self.declare_parameter("startup_on", False)

        self._command_topic = self.get_parameter("command_topic").get_parameter_value().string_value
        self._enabled = False
        self._cmd_pub = self.create_publisher(Bool, self._command_topic, 10)
        self.create_subscription(String, self._INSPECTION_TOPIC, self._on_inspection_event, 10)
        self.get_logger().info(
            "Torch bridge ready (cmd_topic=%s, inspection_topic=%s, startup_on=%s)"
            % (self._command_topic, self._INSPECTION_TOPIC, self.get_parameter("startup_on").value)
        )

        if bool(self.get_parameter("startup_on").value):
            self._publish(True, log=True)

    def _on_inspection_event(self, msg: String) -> None:
        payload = (msg.data or "").strip().upper()
        if payload == "LOW_LIGHT":
            self._publish(True, log=True)
        elif payload == "OK":
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
