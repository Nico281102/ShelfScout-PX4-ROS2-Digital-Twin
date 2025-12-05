"""Metrics node that collects mission KPIs and exports them to CSV."""

from __future__ import annotations

from collections import Counter
from pathlib import Path
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from ..core.kpi import defective_rate, export_csv, export_inspection_log, mission_duration
from ..px4io.qos import EVENTS_QOS
from ..px4io.topics import namespaced


class MetricsNode(Node):
    def __init__(self) -> None:
        super().__init__("mission_metrics")
        self._vehicle_ns = (
            self.declare_parameter("vehicle_ns", "").get_parameter_value().string_value.strip()
        )
        metrics_dir_override = self.declare_parameter("metrics_dir", "").get_parameter_value().string_value
        if metrics_dir_override:
            self._metrics_dir = Path(metrics_dir_override)
        elif self._vehicle_ns:
            self._metrics_dir = Path(f"data/metrics/{self._vehicle_ns}")
        else:
            self._metrics_dir = Path("data/metrics")

        self._inspection_total = 0
        self._inspection_suspect = 0
        self._fallback_counts: Counter[str] = Counter()
        self._mission_start: Optional[float] = None
        self._mission_end: Optional[float] = None
        self._last_state: Optional[str] = None
        self._snapshot_count = 0
        self._snapshot_low_light = 0
        self._inspection_log: list[tuple[int, float, str]] = []
        self._current_snapshot_index = 0

        self.create_subscription(
            String,
            namespaced("overrack/inspection", namespace=self._vehicle_ns),
            self._on_inspection,
            EVENTS_QOS,
        )
        self.create_subscription(
            String,
            namespaced("overrack/mission_state", namespace=self._vehicle_ns),
            self._on_state,
            EVENTS_QOS,
        )

        self._metrics_dir.mkdir(parents=True, exist_ok=True)

    # ------------------------------------------------------------------
    # Callbacks

    def _on_inspection(self, msg: String) -> None:
        payload = (msg.data or "").strip().upper()
        if payload in {"OK", "SUSPECT"}:
            self._inspection_total += 1
            if payload == "SUSPECT":
                self._inspection_suspect += 1
            self._inspection_log.append((self._current_snapshot_index, self._now(), payload))
        elif payload == "LOW_LIGHT":
            self._snapshot_low_light += 1
            self._inspection_log.append((self._current_snapshot_index, self._now(), payload))

    def _on_state(self, msg: String) -> None:
        state_name = msg.data or ""
        now = self._now()
        if self._mission_start is None and state_name not in {"BootstrapState", "OffboardInitState", "ArmingState"}:
            self._mission_start = now
        if state_name == "MISSION_COMPLETE" and self._mission_end is None:
            self._mission_end = now
        if state_name in {"ReturnHomeState", "EmergencyLandState", "FallbackHoldState"}:
            self._fallback_counts[state_name] += 1
        if state_name.lower() == "snapshot":
            self._snapshot_count += 1
            self._current_snapshot_index = self._snapshot_count
        self._last_state = state_name

    # ------------------------------------------------------------------
    # Shutdown hook

    def destroy_node(self) -> bool:
        try:
            self._write_metrics()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"Failed to write metrics: {exc}")
        return super().destroy_node()

    def _write_metrics(self) -> None:
        if self._mission_start is None:
            return
        mission_time = mission_duration(self._mission_start, self._mission_end)
        defect_ratio = defective_rate(self._inspection_total, self._inspection_suspect)
        rows = [
            ("mission_time_s", f"{mission_time:.2f}"),
            ("inspections_total", self._inspection_total),
            ("inspections_suspect", self._inspection_suspect),
            ("defective_rate", f"{defect_ratio:.3f}"),
            ("snapshots_total", self._snapshot_count),
            ("snapshots_low_light", self._snapshot_low_light),
        ]
        for key, count in sorted(self._fallback_counts.items()):
            rows.append((f"fallback_{key}", count))
        if self._last_state:
            rows.append(("last_state", self._last_state))

        outfile = export_csv(rows, self._metrics_dir, "mission_metrics")
        self.get_logger().info(f"Metrics exported to {outfile}")

        detail_file = export_inspection_log(
            self._inspection_log,
            self._metrics_dir,
            "mission_inspections",
            self._mission_start,
        )
        if detail_file is not None:
            self.get_logger().info(f"Inspection log exported to {detail_file}")

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node: Optional[MetricsNode] = None
    try:
        node = MetricsNode()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":  # pragma: no cover
    main()
