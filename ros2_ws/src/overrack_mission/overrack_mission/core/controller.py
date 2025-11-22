"""Mission controller that wires a runtime with the mission FSM."""

from __future__ import annotations

import pathlib
import time
from typing import Optional, Tuple

from ..px4io.telemetry import SpawnSyncState
from .bounds import EnuBounds
from .fsm import MissionContext, MissionRuntime, MissionStateMachine
from .plan import (
    MissionPlan,
    MissionPlanError,
    enforce_cruise_speed_limits,
    load_plan,
    validate_waypoints_in_bounds,
)


class MissionController:
    def __init__(
        self,
        runtime: MissionRuntime,
        mission_path,
        *,
        enu_bounds: Optional[EnuBounds] = None,
        cruise_speed_limits: Optional[Tuple[float, float]] = None,
        debug_frames: bool = False,
        return_home_safe_z: Optional[float] = None,
    ) -> None:
        self._runtime = runtime
        path = pathlib.Path(mission_path)
        try:
            self._plan: MissionPlan = load_plan(path)
        except MissionPlanError as exc:
            runtime.logger.error(f"Mission plan error: {exc}")
            raise

        if cruise_speed_limits is not None:
            enforce_cruise_speed_limits(self._plan, cruise_speed_limits[0], cruise_speed_limits[1])
        if enu_bounds is not None:
            validate_waypoints_in_bounds(self._plan, enu_bounds)

        self._context = MissionContext(
            runtime,
            self._plan,
            debug_frames=debug_frames,
            return_home_safe_z=return_home_safe_z,
        )
        self._state_machine = MissionStateMachine(self._context)
        self._spawn_sync_ready = False
        self._last_wait_log = 0.0
        self._spawn_sync_timeout_s = 5.0

    @property
    def plan(self) -> MissionPlan:
        return self._plan

    def tick(self) -> None:
        self._runtime.setpoints.send_offboard_control_mode()
        if not self._spawn_sync_ready and not self._check_spawn_sync_ready():
            return
        self._state_machine.tick()

    def _check_spawn_sync_ready(self) -> bool:
        telemetry = self._runtime.telemetry
        spawn_state = telemetry.spawn_sync_state()
        spawn_ready = False
        fallback_reason: Optional[str] = None
        now = time.monotonic()

        # If the background thread already signalled completion, refresh the state.
        #if spawn_state is SpawnSyncState.QUERYING and telemetry.spawn_offset_ready_event().is_set():
        #   spawn_state = telemetry.spawn_sync_state()

        if spawn_state is SpawnSyncState.READY:
            spawn_ready = True
        elif spawn_state is SpawnSyncState.FAILED:
            fallback_reason = "query_failed"
        else:
            elapsed = now - telemetry.spawn_sync_state_since()
            if elapsed < self._spawn_sync_timeout_s:
                if spawn_state is SpawnSyncState.UNREQUESTED:
                    self._log_wait_state("Waiting for Gazebo model detection before capturing spawn offset...")
                else:
                    self._log_wait_state("Waiting for Gazebo spawn offset (query in flight)...")
                return False
            fallback_reason = "timeout"

        local = telemetry.local_position()
        if local is None or not getattr(local, "xy_valid", True):
            self._log_wait_state("Waiting for EKF local position...")
            return False
        offset = telemetry.spawn_offset_enu()
        if offset is None:
            self._log_wait_state("Waiting for PX4 to publish initial odometry pose...")
            return False

        if spawn_ready:
            self._runtime.logger.info("Gazebo spawn offset and EKF pose ready; proceeding with TAKEOFF bootstrap")
        else:
            if fallback_reason == "query_failed":
                self._runtime.logger.warn("Gazebo spawn offset query failed; proceeding with PX4 odometry fallback")
            else:
                self._runtime.logger.warn(
                    "Gazebo spawn offset not ready after %.1fs; proceeding with PX4 odometry fallback"
                    % self._spawn_sync_timeout_s
                )
        self._spawn_sync_ready = True
        return True

    def _log_wait_state(self, message: str) -> None:
        now = time.monotonic()
        if (now - self._last_wait_log) >= 1.0:
            self._runtime.logger.info(message)
            self._last_wait_log = now
