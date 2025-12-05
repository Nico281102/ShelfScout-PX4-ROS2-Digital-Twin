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
            self.get_logger().info("No PX4 parameters found in YAML (px4_params.*)")
            self._active = False
            return

        self.get_logger().info(
            f"Waiting for PX4 STANDBY to set parameters: {self._px4_params}"
        )

        self.get_logger().info(
            f"PX4ParamSetter init: vehicle_ns='{self._vehicle_ns}', "
            f"px4_namespace='{self._px4_namespace}', mavlink_url='{self._mavlink_url}'"
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
        self.get_logger().info("PX4 in STANDBY: pushing parameters via MAVLink")
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
            # Support string values that may carry trailing comma from YAML typos.
            if isinstance(value, str):
                value = value.strip().rstrip(",")
            if isinstance(value, (int, float)) and not isinstance(value, bool):
                px4_params[name] = value
            else:
                try:
                    px4_params[name] = float(value)
                except (TypeError, ValueError):
                    self.get_logger().warning(
                        f"PX4 parameter '{name}' ignored: non-numeric type ({type(value).__name__})"
                    )
        return px4_params

    def _run_param_push(self) -> None:
        try:
            asyncio.run(self._set_params_async())
        finally:
            # Lo shutdown del contesto va lasciato al main, evitare doppio shutdown da thread.
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
                getter = drone.param.get_param_float
                cast_value: Numeric = value
                if isinstance(value, bool):
                    setter = drone.param.set_param_int
                    getter = drone.param.get_param_int
                    cast_value = int(value)
                elif isinstance(value, int):
                    setter = drone.param.set_param_int
                    getter = drone.param.get_param_int
                current_before = None
                try:
                    current_before = await getter(name)  # type: ignore[misc]
                except Exception:
                    pass
                await setter(name, cast_value)  # type: ignore[arg-type]
                try:
                    current = await getter(name)  # type: ignore[misc]
                    self.get_logger().info(
                        f"Parameter set: {name} = {cast_value} "
                        f"(before={current_before}, after={current})"
                    )
                except Exception:
                    self.get_logger().info(
                        f"Parameter set: {name} = {cast_value} (before={current_before})"
                    )
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(f"Error setting {name}: {exc}")

        self.get_logger().info("PX4ParamSetter completed -- shutting down")


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
        try:
            rclpy.shutdown()
        except RuntimeError:
            # Già shutdown altrove (ad es. in thread) oppure contesto non inizializzato.
            pass


if __name__ == "__main__":
    main()
