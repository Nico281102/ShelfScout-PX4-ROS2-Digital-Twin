#!/usr/bin/env python3
import asyncio
import threading
from typing import Dict, Optional, Union

import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleStatus

from ..px4io.qos import TELEMETRY_QOS
from ..px4io.topics import namespaced


Numeric = Union[int, float]


class PX4ParamSetter(Node):
    """Set PX4 parameters once the vehicle reaches STANDBY via MAVLink."""

    def __init__(self) -> None:
        super().__init__(
            "px4_param_setter",
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True,
        )

        self._px4_params: Dict[str, Numeric] = self._load_px4_params()
        # Gestione sicura dei parametri che potrebbero essere già presenti come override
        if self.has_parameter("vehicle_ns"):
            self._vehicle_ns = str(self.get_parameter("vehicle_ns").value or "").strip()
        else:
            self._vehicle_ns = ""
            self.declare_parameter("vehicle_ns", self._vehicle_ns)

        if self.has_parameter("px4_namespace"):
            self._px4_namespace = str(self.get_parameter("px4_namespace").value or "").strip().strip("/")
        else:
            self._px4_namespace = ""
            self.declare_parameter("px4_namespace", self._px4_namespace)

        if self.has_parameter("mavlink_url"):
            self._mavlink_url = self.get_parameter("mavlink_url").value
        else:
            self._mavlink_url = "udp://:14540"
            self.declare_parameter("mavlink_url", self._mavlink_url)

        self._connection_thread: Optional[threading.Thread] = None
        self._started = False
        self._active = True

        if not self._px4_params:
            self.get_logger().info("Nessun parametro PX4 trovato nel YAML (px4_params.*)")
            self._active = False
            return

        self.get_logger().info(
            f"In attesa di PX4 STANDBY per impostare i parametri: {self._px4_params}"
        )

        self._status_sub = self.create_subscription(
            VehicleStatus,
            namespaced("fmu/out/vehicle_status", namespace=self._px4_namespace),
            self._on_vehicle_status,
            TELEMETRY_QOS,
        )

    # ------------------------------------------------------------------
    # ROS callbacks

    def _on_vehicle_status(self, msg: VehicleStatus) -> None:
        if self._started:
            return
        if msg.arming_state != VehicleStatus.ARMING_STATE_STANDBY:
            return

        self._started = True
        self.get_logger().info("PX4 in STANDBY: invio parametri via MAVLink")
        self._connection_thread = threading.Thread(
            target=self._run_param_push,
            name="px4_param_setter",
            daemon=True,
        )
        self._connection_thread.start()

    # ------------------------------------------------------------------
    # Helpers

    def _load_px4_params(self) -> Dict[str, Numeric]:
        parameters = self.get_parameters_by_prefix("px4_params")
        px4_params: Dict[str, Numeric] = {}
        for name, param in parameters.items():
            value = param.value
            if isinstance(value, (int, float)) and not isinstance(value, bool):
                px4_params[name] = value
            else:
                try:
                    px4_params[name] = float(value)
                except (TypeError, ValueError):
                    self.get_logger().warn_once(
                        f"Parametro PX4 '{name}' ignorato: tipo non numerico ({type(value).__name__})"
                    )
        return px4_params

    def _run_param_push(self) -> None:
        try:
            asyncio.run(self._set_params_async())
        finally:
            try:
                rclpy.shutdown()
            except Exception:  # noqa: BLE001
                pass

    async def _set_params_async(self) -> None:
        try:
            from mavsdk import System
        except ImportError:
            self.get_logger().error("Modulo mavsdk non disponibile (pip install mavsdk)")
            return

        drone = System()
        self.get_logger().info(f"Connessione MAVSDK a {self._mavlink_url}")
        await drone.connect(system_address=self._mavlink_url)

        async for state in drone.core.connection_state():
            if state.is_connected:
                self.get_logger().info("Connessione MAVSDK stabilita")
                break

        for name, value in self._px4_params.items():
            try:
                setter = drone.param.set_param_float
                cast_value: Numeric = value
                if isinstance(value, bool):
                    setter = drone.param.set_param_int
                    cast_value = int(value)
                elif isinstance(value, int):
                    setter = drone.param.set_param_int
                await setter(name, cast_value)  # type: ignore[arg-type]
                self.get_logger().info(f"Parametro impostato: {name} = {cast_value}")
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(f"Errore impostando {name}: {exc}")

        self.get_logger().info("PX4ParamSetter completato — shutting down")


def main():
    rclpy.init()
    node = PX4ParamSetter()
    try:
        if getattr(node, "_active", True):
            rclpy.spin(node)
    finally:
        if node is not None:
            try:
                node.destroy_node()
            except Exception:
                pass
        rclpy.shutdown()


if __name__ == "__main__":
    main()
