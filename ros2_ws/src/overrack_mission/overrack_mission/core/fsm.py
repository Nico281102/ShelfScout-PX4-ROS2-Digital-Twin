"""Finite state machine driving the mission plan execution (ROS-free)."""

from __future__ import annotations

from abc import ABC, abstractmethod
from collections import deque
from dataclasses import dataclass
from typing import Deque, Final, Optional, Protocol, Sequence

from .plan import FallbackAction, MissionPlan, MissionStep

# PX4 vehicle command codes (mirrors px4_msgs.msg.VehicleCommand)
VEHICLE_CMD_DO_SET_MODE = 176
VEHICLE_CMD_COMPONENT_ARM_DISARM = 400
VEHICLE_CMD_NAV_LAND = 21

# Triggers and action names (centralized to avoid typos)
TRIGGER_BATTERY_WARNING: Final = "battery_warning"
TRIGGER_BATTERY_CRITICAL: Final = "battery_critical"
TRIGGER_LINK_LOST: Final = "link_lost"
TRIGGER_LOW_LIGHT: Final = "low_light"
TRIGGER_INTERNAL: Final = "_internal_"

FALLBACK_PRIORITY: Final[dict[str, int]] = {
    TRIGGER_BATTERY_CRITICAL: 3,
    TRIGGER_LINK_LOST: 2,
    TRIGGER_BATTERY_WARNING: 1,
    TRIGGER_LOW_LIGHT: 0,
    TRIGGER_INTERNAL: -1,
}

ACTION_RETURN_HOME: Final = "return_home"
ACTION_LAND: Final = "land"
ACTION_HOLD: Final = "hold"
ACTION_INCREASE_HOVER: Final = "increase_hover"


@dataclass
class LocalPosition:
    x: float
    y: float
    z: float
    stamp_s: float
    xy_valid: bool = True
    z_valid: bool = True


class FallbackSequence:
    """Atomic, ordered fallback action sequence tied to a trigger."""

    def __init__(self, trigger: str, actions: Sequence[FallbackAction], priority: int) -> None:
        self.trigger = trigger
        self.priority = priority
        self._actions: Deque[FallbackAction] = deque(actions)
        self._current_action: Optional[FallbackAction] = None

    def next_action(self) -> Optional[FallbackAction]:
        if self._current_action is None:
            if not self._actions:
                return None
            self._current_action = self._actions.popleft()
        return self._current_action

    def complete_action(self) -> None:
        self._current_action = None

    def append_action(self, action: FallbackAction) -> None:
        self._actions.append(action)

    def done(self) -> bool:
        return self._current_action is None and not self._actions


def _trigger_priority(trigger: str) -> int:
    return FALLBACK_PRIORITY.get(trigger, -1)


class LoggerLike(Protocol):
    def info(self, msg: str) -> None: ...

    def warn(self, msg: str) -> None: ...

    def error(self, msg: str) -> None: ...

    def debug(self, msg: str) -> None: ...


class SetpointPublisherProto(Protocol):
    def send_offboard_control_mode(self) -> None: ...

    def send_trajectory_setpoint(self, position: Sequence[float], yaw_deg: float = 0.0) -> None: ...

    def send_vehicle_command(self, command: int, *, param1: float = 0.0, param2: float = 0.0) -> None: ...

    def describe_target(
        self, position: Sequence[float], yaw_deg: float = 0.0
    ) -> tuple[tuple[float, float, float], tuple[float, float, float], float]: ...


class TelemetryProto(Protocol):
    def local_position(self) -> Optional[LocalPosition]: ...

    def vehicle_status(self): ...  # pragma: no cover - passthrough for compatibility

    def battery_status(self): ...  # pragma: no cover - passthrough for compatibility

    def is_offboard_active(self) -> bool: ...

    def is_armed(self) -> bool: ...

    def data_link_ok(self) -> bool: ...

    def preflight_checks_pass(self) -> bool: ...

    def battery_warning(self) -> bool: ...

    def battery_critical(self) -> bool: ...

    def pop_inspection_result(self) -> Optional[str]: ...

    def consume_low_light_event(self) -> bool: ...

    def log_waiting_preflight(self) -> None: ...

    def spawn_offset_enu(self) -> Optional[tuple[float, float, float]]: ...  # pragma: no cover - passthrough


class MissionRuntime(Protocol):
    @property
    def logger(self) -> LoggerLike: ...

    def now(self) -> float: ...  # seconds

    def publish_state(self, name: str) -> None: ...

    @property
    def telemetry(self) -> TelemetryProto: ...

    @property
    def setpoints(self) -> SetpointPublisherProto: ...


@dataclass
class MissionContext:
    runtime: MissionRuntime
    plan: MissionPlan
    debug_frames: bool = False
    return_home_safe_z: Optional[float] = None

    def __post_init__(self) -> None:
        self.telemetry = self.runtime.telemetry
        self.setpoints = self.runtime.setpoints
        self.current_index = 0
        self.hover_deadline: Optional[float] = None
        self._hover_extension_s = 0.0
        self._last_hover_duration_s = self.plan.hover_time_s
        self._active_sequence: Optional[FallbackSequence] = None
        self.hold_until: Optional[float] = None
        self._debug_frames = bool(self.debug_frames)
        self._bootstrap_complete = False
        self._hold_override_enu: Optional[tuple[float, float, float]] = None

    def hover_elapsed(self) -> bool:
        return self.hover_deadline is not None and self.now() >= self.hover_deadline

    # ------------------------------------------------------------------
    # Convenience helpers

    @property
    def logger(self) -> LoggerLike:
        return self.runtime.logger

    def now(self) -> float:
        return self.runtime.now()

    @property
    def target_position_enu(self) -> tuple[float, float, float]:
        step = self.current_step
        return (step.position[0], step.position[1], step.position[2])

    def current_position_enu(self) -> Optional[tuple[float, float, float]]:
        """Compute current ENU position from PX4 local NED + spawn offset."""
        local = self.telemetry.local_position()
        if local is None:
            return None
        offset = self.telemetry.spawn_offset_enu() or (0.0, 0.0, 0.0)
        # local NED -> ENU: x_ned (north) -> y_enu, y_ned (east) -> x_enu, z_ned (down) -> -z_enu
        enu_x = local.y + offset[0]
        enu_y = local.x + offset[1]
        enu_z = -local.z + offset[2]
        return (enu_x, enu_y, enu_z)

    def hold_position_enu(self) -> tuple[float, float, float]:
        """Return the position used while holding; defaults to current waypoint unless overridden (e.g. return_home)."""
        return self._hold_override_enu or self.target_position_enu

    def set_hold_override(self, enu: tuple[float, float, float]) -> None:
        self._hold_override_enu = (float(enu[0]), float(enu[1]), float(enu[2]))

    def clear_hold_override(self) -> None:
        self._hold_override_enu = None

    def bootstrap_position_enu(self) -> tuple[float, float, float]:
        if not self.bootstrap_complete:
            return (0.0, 0.0, self.plan.altitude_m)
        offset = self.telemetry.spawn_offset_enu() or (0.0, 0.0, 0.0)
        return (offset[0], offset[1], self.plan.altitude_m)

    def home_position_enu(self) -> tuple[float, float, float]:
        offset = self.telemetry.spawn_offset_enu() or (0.0, 0.0, 0.0)
        return (offset[0], offset[1], self.plan.altitude_m)

    @property
    def bootstrap_complete(self) -> bool:
        return self._bootstrap_complete

    def mark_bootstrap_complete(self) -> None:
        if self._bootstrap_complete:
            return
        self._bootstrap_complete = True
        try:
            self.setpoints.set_bootstrap_mode(False)
        except AttributeError:
            pass

    def current_ned_target(self) -> tuple[float, float, float]:
        ned_target, _, _ = self.setpoints.describe_target(self.target_position_enu, self.current_step.yaw_deg)
        return ned_target

    @property
    def target_yaw_deg(self) -> float:
        return self.current_step.yaw_deg

    @property
    def current_step(self) -> MissionStep:
        return self.plan.steps[self.current_index]

    def tolerance_xy(self) -> float:
        return 0.25

    def tolerance_3d(self) -> float:
        return 0.25

    def tolerance_z(self) -> float:
        return 0.25

    def advance_waypoint(self) -> bool:
        if self.current_index < len(self.plan.steps) - 1:
            self.current_index += 1
            if self.current_index == 1 and not self._bootstrap_complete:
                self.mark_bootstrap_complete()
            return True
        self.publish_state("MISSION_COMPLETE")
        if not self._bootstrap_complete:
            self.mark_bootstrap_complete()
        return False

    def last_hover_duration(self) -> float:
        return self._last_hover_duration_s

    def log_frame_debug(self, label: str) -> None:
        if not self._debug_frames:
            return
        local = self.telemetry.local_position()
        if local is None:
            return
        ned_target, origin, yaw_rad = self.setpoints.describe_target(self.target_position_enu, self.current_step.yaw_deg)
        enu = self.target_position_enu
        message = (
            f"FrameDebug[{label}] wp={self.current_index} "
            f"ENU=({enu[0]:.2f}, {enu[1]:.2f}, {enu[2]:.2f}) "
            f"NED=({ned_target[0]:.2f}, {ned_target[1]:.2f}, {ned_target[2]:.2f}) "
            f"origin=({origin[0]:.2f}, {origin[1]:.2f}, {origin[2]:.2f}) "
            f"local=({local.x:.2f}, {local.y:.2f}, {local.z:.2f}) "
            f"yaw={yaw_rad:.2f} rad"
        )
        self.logger.info(message)

    def send_position_setpoint(self, position: Sequence[float], yaw_deg: float, *, source: str) -> None:
        target_enu = (float(position[0]), float(position[1]), float(position[2]))
        ned_target, origin_ned, yaw_rad = self.setpoints.describe_target(target_enu, yaw_deg)
        origin_enu = self.telemetry.spawn_offset_enu() or (0.0, 0.0, 0.0)
        local = self.telemetry.local_position()
        dist_info = ""
        if local is not None:
            dx = ned_target[0] - local.x
            dy = ned_target[1] - local.y
            dz = ned_target[2] - local.z
            dist_xy = (dx ** 2 + dy ** 2) ** 0.5
            dist_3d = (dx ** 2 + dy ** 2 + dz ** 2) ** 0.5
            dist_info = (
                f" dist3d={dist_3d:.2f}m dist_xy={dist_xy:.2f}m dz={abs(dz):.2f}m"
            )
        self.logger.info(
            "Setpoint[%s]: target_enu=(%.2f, %.2f, %.2f) origin_enu=(%.2f, %.2f, %.2f) "
            "target_ned=(%.2f, %.2f, %.2f) origin_ned=(%.2f, %.2f, %.2f) yaw=%.1fdeg%s"
            % (
                source,
                target_enu[0],
                target_enu[1],
                target_enu[2],
                origin_enu[0],
                origin_enu[1],
                origin_enu[2],
                ned_target[0],
                ned_target[1],
                ned_target[2],
                origin_ned[0],
                origin_ned[1],
                origin_ned[2],
                yaw_deg,
                dist_info,
            )
        )
        self.setpoints.send_trajectory_setpoint(target_enu, yaw_deg)

    # Hover helpers -----------------------------------------------------

    def add_hover_extension(self, seconds: Optional[float]) -> None:
        if seconds is None:
            return
        extension_target = self.now() + seconds
        self._hover_extension_s = max(self._hover_extension_s, seconds)
        if self.hover_deadline is None or extension_target > self.hover_deadline:
            if self.hover_deadline is not None:
                self._last_hover_duration_s += extension_target - self.hover_deadline
            self.hover_deadline = extension_target

    def schedule_hover_deadline(self) -> float:
        base = self.current_step.hover_override_s
        if base is None:
            base = self.plan.hover_time_s
        duration = base + self._hover_extension_s
        self._hover_extension_s = 0.0
        self._last_hover_duration_s = duration
        return self.now() + duration

    # Fallback handling -------------------------------------------------

    @property
    def active_sequence(self) -> Optional[FallbackSequence]:
        return self._active_sequence

    def start_fallback_sequence(self, trigger: str, actions: Sequence[FallbackAction]) -> None:
        if not actions or self._active_sequence is not None:
            return
        priority = _trigger_priority(trigger)
        self._active_sequence = FallbackSequence(trigger, actions, priority)
        action_names = ", ".join(a.name for a in actions)
        self.logger.info(f"Fallback sequence started for '{trigger}' with actions: {action_names}")
        self.hold_until = None

    def override_fallback_sequence(self, trigger: str, actions: Sequence[FallbackAction]) -> None:
        if not actions:
            return
        priority = _trigger_priority(trigger)
        previous = self._active_sequence.trigger if self._active_sequence else None
        self._active_sequence = FallbackSequence(trigger, actions, priority)
        self.hold_until = None
        action_names = ", ".join(a.name for a in actions)
        if previous:
            self.logger.warn(
                f"Fallback sequence overridden: '{previous}' -> '{trigger}' (actions: {action_names})"
            )
        else:
            self.logger.info(f"Fallback sequence started for '{trigger}' with actions: {action_names}")

    def active_fallback_action(self) -> Optional[FallbackAction]:
        if self._active_sequence is None:
            return None
        return self._active_sequence.next_action()

    def complete_active_action(self) -> None:
        if self._active_sequence is None:
            return
        self._active_sequence.complete_action()
        self.hold_until = None
        if self._active_sequence.done():
            self.logger.info(f"Fallback sequence '{self._active_sequence.trigger}' complete")
            self._active_sequence = None

    def next_fallback_state(self, *, current_state: Optional["State"] = None) -> Optional[type["State"]]:
        sequence = self._active_sequence
        if sequence is None:
            return None
        while True:
            action = sequence.next_action()
            if action is None:
                self.logger.info(f"Fallback sequence '{sequence.trigger}' complete")
                self._active_sequence = None
                return None
            action_name = action.name.lower()
            if action_name == ACTION_INCREASE_HOVER:
                self.add_hover_extension(action.value or self.plan.hover_time_s)
                self.complete_active_action()
                continue
            if action_name == ACTION_RETURN_HOME:
                return ReturnHomeState
            if action_name == ACTION_LAND:
                return EmergencyLandState
            if action_name == ACTION_HOLD:
                duration = action.value or self.plan.hover_time_s
                self.hold_until = self.now() + duration
                return HoldState
            self.logger.warn(f"Unknown fallback action '{action.name}', skipping")
            self.complete_active_action()

    def queue_action(self, name: str, value: Optional[float] = None, *, trigger: str = TRIGGER_INTERNAL) -> None:
        action = FallbackAction(name, value)
        if self._active_sequence is None:
            self.start_fallback_sequence(trigger, [action])
            return
        if self._active_sequence.trigger == trigger:
            self._active_sequence.append_action(action)
            return
        if _trigger_priority(trigger) > self._active_sequence.priority:
            self.override_fallback_sequence(trigger, [action])
            return
        self.logger.warn(
            f"Ignored queued action '{name}' for trigger '{trigger}' because "
            f"fallback sequence '{self._active_sequence.trigger}' is active"
        )

    # State publication -------------------------------------------------

    def publish_state(self, name: str) -> None:
        self.runtime.publish_state(name)


class State(ABC):
    """Lifecycle interface for FSM states. Keep persistent data in MissionContext."""

    def enter(self, ctx: MissionContext) -> None:  # pragma: no cover - default noop
        pass

    def body(self, ctx: MissionContext) -> None:  # pragma: no cover - default noop
        pass

    @abstractmethod
    def tick(self, ctx: MissionContext) -> Optional[type["State"]]:
        """Return the next state class or None to remain."""
        raise NotImplementedError

    def exit(self, ctx: MissionContext) -> None:  # pragma: no cover - default noop
        pass


class BootstrapState(State):
    def __init__(self) -> None:
        self.counter = 0

    def body(self, ctx: MissionContext) -> None:
        ctx.send_position_setpoint(ctx.bootstrap_position_enu(), ctx.target_yaw_deg, source="BootstrapState")

    def tick(self, ctx: MissionContext) -> Optional[type[State]]:
        self.counter += 1
        return OffboardInitState if self.counter >= 20 else None


class OffboardInitState(State):
    def __init__(self) -> None:
        self._command_sent = False

    def body(self, ctx: MissionContext) -> None:
        ctx.send_position_setpoint(ctx.bootstrap_position_enu(), ctx.target_yaw_deg, source="OffboardInitState")

    def tick(self, ctx: MissionContext) -> Optional[type[State]]:
        if not self._command_sent:
            ctx.logger.info("OFFBOARD mode command sent")
            ctx.setpoints.send_vehicle_command(
                VEHICLE_CMD_DO_SET_MODE,
                param1=1.0,
                param2=6.0,
            )
            self._command_sent = True
            return None
        if ctx.telemetry.is_offboard_active():
            return ArmingState
        return None


class ArmingState(State):
    def __init__(self) -> None:
        self._arm_sent = False

    def body(self, ctx: MissionContext) -> None:
        ctx.send_position_setpoint(ctx.bootstrap_position_enu(), ctx.target_yaw_deg, source="ArmingState")

    def tick(self, ctx: MissionContext) -> Optional[type[State]]:
        if not ctx.telemetry.is_offboard_active():
            return None
        if not ctx.telemetry.preflight_checks_pass():
            ctx.telemetry.log_waiting_preflight()
            return None
        if not self._arm_sent:
            ctx.logger.info("ARM command sent")
            ctx.setpoints.send_vehicle_command(
                VEHICLE_CMD_COMPONENT_ARM_DISARM,
                param1=1.0,
            )
            self._arm_sent = True
            return None
        if ctx.telemetry.is_armed():
            return TransitState
        return None


class TransitState(State):
    def enter(self, ctx: MissionContext) -> None:
        ctx.hover_deadline = None

    def body(self, ctx: MissionContext) -> None:
        ctx.send_position_setpoint(ctx.target_position_enu, ctx.target_yaw_deg, source="TransitState")

    def tick(self, ctx: MissionContext) -> Optional[type[State]]:
        local = ctx.telemetry.local_position()
        if local is None:
            return None
        target = ctx.current_ned_target()
        dx = target[0] - local.x
        dy = target[1] - local.y
        dz = target[2] - local.z
        distance_xy = (dx ** 2 + dy ** 2) ** 0.5
        altitude_error = abs(dz)
        distance_3d = (dx ** 2 + dy ** 2 + dz ** 2) ** 0.5
        if distance_3d >= ctx.tolerance_3d() or altitude_error >= ctx.tolerance_z():
            ctx.logger.info(
                "Transit remaining: dist3d=%.2f xy=%.2f dz=%.2f tol3d=%.2f ztol=%.2f local=(%.2f, %.2f, %.2f) target=(%.2f, %.2f, %.2f)"
                % (
                    distance_3d,
                    distance_xy,
                    altitude_error,
                    ctx.tolerance_3d(),
                    ctx.tolerance_z(),
                    local.x,
                    local.y,
                    local.z,
                    target[0],
                    target[1],
                    target[2],
                )
            )
        if distance_3d < ctx.tolerance_3d() and altitude_error < ctx.tolerance_z():
            step = ctx.current_step
            ctx.logger.info(
                "FSM state -> HOVER, wp_idx=%d, target=(%.2f, %.2f, %.2f)"
                % (ctx.current_index, target[0], target[1], target[2])
            )
            ctx.hover_deadline = ctx.schedule_hover_deadline()
            ctx.logger.info(
                "Waypoint %d/%d reached. Hovering for %.1fs"
                % (
                    ctx.current_index + 1,
                    len(ctx.plan.steps),
                    ctx.last_hover_duration(),
                )
            )
            s = ctx.telemetry.battery_status()
            if s:
                ctx.logger.info(
                    f"[Battery] At waypoint {ctx.current_index+1}: "
                    f"{s.remaining*100:.1f}% remaining, warning={s.warning}"
                )
            ctx.log_frame_debug("waypoint")
            if step.inspect:
                return InspectState
            return HoverState
        return None


class HoverState(State):
    def body(self, ctx: MissionContext) -> None:
        ctx.send_position_setpoint(ctx.target_position_enu, ctx.target_yaw_deg, source="HoverState")

    def tick(self, ctx: MissionContext) -> Optional[type[State]]:
        if ctx.hover_deadline is None:
            return TransitState
        if ctx.hover_elapsed():
            if ctx.advance_waypoint():
                next_target = ctx.current_ned_target()
                ctx.logger.info(
                    "FSM state -> TRANSIT, wp_idx=%d, target=(%.2f, %.2f, %.2f)"
                    % (ctx.current_index, next_target[0], next_target[1], next_target[2])
                )
                return TransitState
            ctx.logger.info("FSM state -> HOLD, mission complete. Holding last setpoint (hover)")
            if ctx.plan.land_on_finish:
                ctx.queue_action(ACTION_RETURN_HOME)
                ctx.queue_action(ACTION_LAND)
            return HoldState
        return None


class InspectState(State):
    def __init__(self) -> None:
        self._deadline: Optional[float] = None
        self._wait_for_ack = False

    def enter(self, ctx: MissionContext) -> None:
        self._wait_for_ack = ctx.plan.inspection.require_ack
        timeout = ctx.plan.inspection.timeout_s
        if self._wait_for_ack:
            self._deadline = ctx.now() + timeout
            ctx.logger.info("FSM state -> INSPECT (timeout %.1fs)" % timeout)
        else:
            self._deadline = None
            ctx.logger.info("FSM state -> INSPECT (hover-only)")

    def body(self, ctx: MissionContext) -> None:
        ctx.send_position_setpoint(ctx.target_position_enu, ctx.target_yaw_deg, source="InspectState")

    def tick(self, ctx: MissionContext) -> Optional[type[State]]:
        if self._wait_for_ack:
            result = ctx.telemetry.pop_inspection_result()
            if result:
                ctx.logger.info(
                    "Inspection result at waypoint %d: %s"
                    % (ctx.current_index + 1, result)
                )
                return self._advance(ctx)
            if self._deadline is not None and ctx.now() >= self._deadline:
                ctx.logger.warn(
                    "Inspection timeout at waypoint %d"
                    % (ctx.current_index + 1)
                )
                return self._advance(ctx)
        else:
            return self._advance_on_hover(ctx)
        return None

    def _advance_on_hover(self, ctx: MissionContext) -> Optional[type[State]]:
        if ctx.hover_deadline is None:
            return TransitState
        if ctx.hover_elapsed():
            return self._advance(ctx)
        return None

    def _advance(self, ctx: MissionContext) -> Optional[type[State]]:
        if ctx.advance_waypoint():
            next_target = ctx.current_ned_target()
            ctx.logger.info(
                "FSM state -> TRANSIT, wp_idx=%d, target=(%.2f, %.2f, %.2f)"
                % (ctx.current_index, next_target[0], next_target[1], next_target[2])
            )
            return TransitState
        ctx.logger.info("FSM state -> HOLD, mission complete. Holding last setpoint (hover)")
        if ctx.plan.land_on_finish:
            ctx.queue_action(ACTION_RETURN_HOME)
            ctx.queue_action(ACTION_LAND)
        return HoldState


class HoldState(State):
    def __init__(self) -> None:
        self._hold_target: Optional[tuple[float, float, float]] = None

    def enter(self, ctx: MissionContext) -> None:
        self._hold_target = ctx.current_position_enu() or ctx.hold_position_enu()

    def body(self, ctx: MissionContext) -> None:
        target = self._hold_target or ctx.hold_position_enu()
        ctx.send_position_setpoint(target, ctx.target_yaw_deg, source="HoldState")

    def tick(self, ctx: MissionContext) -> Optional[type[State]]:
        if ctx.hold_until is not None and ctx.now() >= ctx.hold_until:
            ctx.logger.info("Timed hold complete")
            ctx.complete_active_action()
            # Let next_fallback_state decide if another action should take over.
        return None


class ReturnHomeState(State):
    def __init__(self) -> None:
        self._last_target_enu: Optional[tuple[float, float, float]] = None
        self._phase: Optional[str] = None

    def body(self, ctx: MissionContext) -> None:
        home_enu = ctx.home_position_enu()
        safe_z = ctx.return_home_safe_z
        current = ctx.current_position_enu()

        def choose_initial_phase() -> str:
            if current is None:
                return "climb"
            if safe_z is None:
                return "final"
            if current[2] < safe_z - ctx.tolerance_z():
                return "climb"
            if (
                abs(home_enu[0] - current[0]) > ctx.tolerance_xy()
                or abs(home_enu[1] - current[1]) > ctx.tolerance_xy()
            ):
                return "translate"
            if current[2] > home_enu[2] + ctx.tolerance_z():
                return "descend"
            return "final"

        if self._phase is None:
            self._phase = choose_initial_phase()

        target_enu = home_enu
        if safe_z is not None:
            if self._phase == "climb":
                # Climb/descend vertically to safe_z at current XY
                xy = (current[0], current[1]) if current is not None else (home_enu[0], home_enu[1])
                target_enu = (xy[0], xy[1], safe_z)
                if current is not None and current[2] >= safe_z - ctx.tolerance_z():
                    if (
                        abs(home_enu[0] - current[0]) > ctx.tolerance_xy()
                        or abs(home_enu[1] - current[1]) > ctx.tolerance_xy()
                    ):
                        self._phase = "translate"
                    elif current[2] > home_enu[2] + ctx.tolerance_z():
                        self._phase = "descend"
                    else:
                        self._phase = "final"
            elif self._phase == "translate":
                # Translate to home XY at safe_z
                target_enu = (home_enu[0], home_enu[1], safe_z)
                if current is not None:
                    if (
                        abs(home_enu[0] - current[0]) <= ctx.tolerance_xy()
                        and abs(home_enu[1] - current[1]) <= ctx.tolerance_xy()
                    ):
                        self._phase = "descend" if current[2] > home_enu[2] + ctx.tolerance_z() else "final"
            elif self._phase == "descend":
                # Descend to home altitude
                target_enu = home_enu
                if current is not None and current[2] <= home_enu[2] + ctx.tolerance_z():
                    self._phase = "final"
            else:
                target_enu = home_enu

        self._last_target_enu = target_enu
        ctx.send_position_setpoint(target_enu, 0.0, source="ReturnHomeState")

    def tick(self, ctx: MissionContext) -> Optional[type[State]]:
        target_enu = self._last_target_enu or ctx.home_position_enu()
        home_enu = ctx.home_position_enu()
        local = ctx.telemetry.local_position()
        if local is None:
            return None
        ned_target, _, _ = ctx.setpoints.describe_target(target_enu, 0.0)
        dx = ned_target[0] - local.x
        dy = ned_target[1] - local.y
        dz = ned_target[2] - local.z
        distance_xy = (dx ** 2 + dy ** 2) ** 0.5
        altitude_error = abs(dz)
        if self._phase != "final":
            return None
        # Evaluate completion only against the final home target to avoid
        # stopping at intermediate climb/translate waypoints.
        ned_home, _, _ = ctx.setpoints.describe_target(home_enu, 0.0)
        hx = ned_home[0] - local.x
        hy = ned_home[1] - local.y
        hz = ned_home[2] - local.z
        home_xy = (hx ** 2 + hy ** 2) ** 0.5
        home_alt_err = abs(hz)
        if home_xy < ctx.tolerance_xy() and home_alt_err < ctx.tolerance_z():
            ctx.logger.info("Return-home reached. Holding position")
            ctx.set_hold_override(ctx.home_position_enu())
            ctx.complete_active_action()
            next_state = ctx.next_fallback_state(current_state=self)
            if next_state:
                return next_state
            return HoldState
        return None


class EmergencyLandState(State):
    def __init__(self) -> None:
        self._command_sent = False

    def enter(self, ctx: MissionContext) -> None:
        ctx.logger.error("Fallback engaged: EMERGENCY LAND")
        ctx.setpoints.send_vehicle_command(VEHICLE_CMD_NAV_LAND)
        self._command_sent = True
        ctx.complete_active_action()

    def tick(self, ctx: MissionContext) -> Optional[type[State]]:
        if not ctx.telemetry.is_armed(): # When PX4 finishes NAV_LAND it disarms automatically.
            ctx.logger.info("Landing complete. Transitioning to MissionEndState")
            return MissionEndState
        return None

class MissionEndState(State):
    def tick(self, ctx):
        return None   # Do nothing forever


class FallbackEvaluator:
    """Centralized fallback trigger handling and action selection."""

    def __init__(self, ctx: MissionContext) -> None:
        self._ctx = ctx
        self._latched_triggers: set[str] = set()

    def process_external_events(self) -> None:
        telem = self._ctx.telemetry
        newly_active: list[str] = []
        current_active: set[str] = set()

        def detect(trigger: str, active: bool) -> None:
            if not active:
                return
            current_active.add(trigger)
            if trigger not in self._latched_triggers:
                newly_active.append(trigger)

        detect(TRIGGER_BATTERY_CRITICAL, telem.battery_critical())
        detect(TRIGGER_BATTERY_WARNING, telem.battery_warning())
        detect(TRIGGER_LINK_LOST, not telem.data_link_ok())
        if telem.consume_low_light_event():
            newly_active.append(TRIGGER_LOW_LIGHT)

        # Update latches for next tick (low_light is event-based, so we do not latch it)
        self._latched_triggers = {t for t in current_active if t != TRIGGER_LOW_LIGHT}

        # Handle low-light as non-interrupting hover extension
        if TRIGGER_LOW_LIGHT in newly_active:
            actions = self._ctx.plan.fallback.get(TRIGGER_LOW_LIGHT) or []
            for action in actions:
                if action.name.lower() == ACTION_INCREASE_HOVER:
                    self._ctx.add_hover_extension(action.value or self._ctx.plan.hover_time_s)
                else:
                    self._ctx.logger.warn(
                        f"Low-light fallback action '{action.name}' ignored (non-interrupting only)"
                    )
            newly_active = [t for t in newly_active if t != TRIGGER_LOW_LIGHT]

        if not newly_active:
            return

        # Highest-priority trigger with configured actions wins for sequence start/override
        for trigger in sorted(newly_active, key=_trigger_priority, reverse=True):
            priority = _trigger_priority(trigger)
            actions = self._ctx.plan.fallback.get(trigger)
            if trigger == TRIGGER_BATTERY_CRITICAL and not actions:
                self._ctx.logger.warn(
                    "Battery critical detected without configured fallback; using emergency LAND"
                )
                actions = [FallbackAction(ACTION_LAND)]
            if not actions:
                continue

            active_seq = self._ctx.active_sequence
            if active_seq is None:
                self._ctx.start_fallback_sequence(trigger, actions)
                return
            if trigger == active_seq.trigger:
                return
            if priority > active_seq.priority:
                self._ctx.override_fallback_sequence(trigger, actions)
            return

    def next_state(self, current_state: Optional[State] = None) -> Optional[type[State]]:
        return self._ctx.next_fallback_state(current_state=current_state)


class MissionStateMachine:
    def __init__(self, ctx: MissionContext, fallback: Optional[FallbackEvaluator] = None) -> None:
        self._ctx = ctx
        self._fallback = fallback or FallbackEvaluator(ctx)
        self._state: State = BootstrapState()
        self._state.enter(ctx)
        ctx.publish_state(self._state.__class__.__name__)

    def tick(self) -> None:
        ctx = self._ctx
        if isinstance(self._state, MissionEndState):
            return
        self._fallback.process_external_events()
        fallback_state = self._fallback.next_state(self._state)
        if fallback_state is not None and not isinstance(self._state, fallback_state):
            self._transition(fallback_state)
            return

        self._state.body(ctx)

        next_state_cls = self._state.tick(ctx)
        if next_state_cls is not None:
            self._handle_step_action_transition(ctx, next_state_cls)
            self._transition(next_state_cls)
            fallback_state = self._fallback.next_state(self._state)
            if fallback_state is not None and not isinstance(self._state, fallback_state):
                self._transition(fallback_state)
            return

        fallback_state = self._fallback.next_state(self._state)
        if fallback_state is not None and not isinstance(self._state, fallback_state):
            self._transition(fallback_state)

    def _handle_step_action_transition(self, ctx: MissionContext, next_state_cls: type[State]) -> None:
        if isinstance(self._state, (HoverState, InspectState)) and ctx.current_step.action:
            action = ctx.current_step.action
            ctx.logger.info(f"Mission action trigger: {action} at step {ctx.current_index}")
            ctx.publish_state(action)

    def _transition(self, state_cls: type[State]) -> None:
        self._state.exit(self._ctx)
        self._state = state_cls()
        self._state.enter(self._ctx)
        self._ctx.publish_state(self._state.__class__.__name__)


__all__ = [
    "MissionRuntime",
    "MissionContext",
    "MissionStateMachine",
    "FallbackEvaluator",
    "State",
    "LocalPosition",
]
