"""Finite state machine driving the mission plan execution (ROS-free)."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from typing import Deque, Optional, Protocol, Sequence, Type

from .plan import FallbackAction, MissionPlan, MissionStep

# PX4 vehicle command codes (mirrors px4_msgs.msg.VehicleCommand)
VEHICLE_CMD_DO_SET_MODE = 176
VEHICLE_CMD_COMPONENT_ARM_DISARM = 400
VEHICLE_CMD_NAV_LAND = 21


@dataclass
class LocalPosition:
    x: float
    y: float
    z: float
    stamp_s: float
    xy_valid: bool = True
    z_valid: bool = True


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

    def __post_init__(self) -> None:
        self.telemetry = self.runtime.telemetry
        self.setpoints = self.runtime.setpoints
        self.current_index = 0
        self.hover_deadline: Optional[float] = None
        self._hover_extension_s = 0.0
        self._last_hover_duration_s = self.plan.hover_time_s
        self._mission_complete = False
        self._mission_start_time: Optional[float] = None
        self._mission_end_time: Optional[float] = None
        self._fallback_queue: Deque[FallbackAction] = deque()
        self._active_action: Optional[FallbackAction] = None
        self._handled_triggers: set[str] = set()
        self.hold_until: Optional[float] = None
        self._debug_frames = bool(self.debug_frames)
        self._bootstrap_complete = False

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
        return max(0.25, self.plan.cruise_speed_mps * 0.25)

    def tolerance_z(self) -> float:
        return 0.3

    def advance_waypoint(self) -> bool:
        if self.current_index < len(self.plan.steps) - 1:
            self.current_index += 1
            if self.current_index == 1 and not self._bootstrap_complete:
                self.mark_bootstrap_complete()
            return True
        self._mission_complete = True
        self._mission_end_time = self.now()
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

    # Mission lifecycle -------------------------------------------------

    def mark_mission_started(self) -> None:
        if self._mission_start_time is None:
            self._mission_start_time = self.now()

    def mission_complete(self) -> bool:
        return self._mission_complete

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

    def enqueue_fallback(self, trigger: str) -> None:
        actions = self.plan.fallback.get(trigger)
        if not actions:
            return
        if trigger != "low_light" and trigger in self._handled_triggers:
            return
        if trigger != "low_light":
            self._handled_triggers.add(trigger)
        for action in actions:
            self._fallback_queue.append(action)

    def next_fallback_state(self) -> Optional[Type["State"]]:
        while self._active_action is None and self._fallback_queue:
            action = self._fallback_queue.popleft()
            action_name = action.name.lower()
            if action_name == "increase_hover":
                self.add_hover_extension(action.value or self.plan.hover_time_s)
                continue
            if action_name == "return_home":
                self._active_action = action
                return ReturnHomeState
            if action_name == "land":
                self._active_action = action
                return EmergencyLandState
            if action_name == "hold":
                duration = action.value or self.plan.hover_time_s
                self.hold_until = self.now() + duration
                self._active_action = action
                return FallbackHoldState
            self.logger.warn(f"Unknown fallback action '{action.name}', ignoring")
        return None

    def active_action(self) -> Optional[FallbackAction]:
        return self._active_action

    def complete_active_action(self) -> None:
        self._active_action = None
        self.hold_until = None

    def has_pending_actions(self) -> bool:
        if self._active_action is not None:
            return True
        return bool(self._fallback_queue)

    def queue_action(self, name: str, value: Optional[float] = None) -> None:
        self._fallback_queue.append(FallbackAction(name, value))

    # State publication -------------------------------------------------

    def publish_state(self, name: str) -> None:
        self.runtime.publish_state(name)


class State:
    """Interface for FSM states."""

    def on_enter(self, ctx: MissionContext) -> None:  # pragma: no cover - default noop
        pass

    def on_exit(self, ctx: MissionContext) -> None:  # pragma: no cover - default noop
        pass

    def tick(self, ctx: MissionContext) -> Optional[Type["State"]]:
        raise NotImplementedError


class BootstrapState(State):
    def __init__(self) -> None:
        self.counter = 0

    def tick(self, ctx: MissionContext) -> Optional[Type[State]]:
        ctx.setpoints.send_trajectory_setpoint(ctx.bootstrap_position_enu(), ctx.target_yaw_deg)
        self.counter += 1
        if self.counter >= 20:
            return OffboardInitState
        return None


class OffboardInitState(State):
    def __init__(self) -> None:
        self._command_sent = False

    def tick(self, ctx: MissionContext) -> Optional[Type[State]]:
        ctx.setpoints.send_trajectory_setpoint(ctx.bootstrap_position_enu(), ctx.target_yaw_deg)
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

    def tick(self, ctx: MissionContext) -> Optional[Type[State]]:
        ctx.setpoints.send_trajectory_setpoint(ctx.bootstrap_position_enu(), ctx.target_yaw_deg)
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
            ctx.mark_mission_started()
            return TransitState
        return None


class TransitState(State):
    def tick(self, ctx: MissionContext) -> Optional[Type[State]]:
        ctx.setpoints.send_trajectory_setpoint(ctx.target_position_enu, ctx.target_yaw_deg)
        ctx.hover_deadline = None
        local = ctx.telemetry.local_position()
        if local is None:
            return None
        target = ctx.current_ned_target()
        dx = target[0] - local.x
        dy = target[1] - local.y
        dz = target[2] - local.z
        distance_xy = (dx ** 2 + dy ** 2) ** 0.5
        altitude_error = abs(dz)
        if distance_xy < ctx.tolerance_xy() and altitude_error < ctx.tolerance_z():
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
            ctx.log_frame_debug("waypoint")
            if step.inspect:
                return InspectState
            return HoverState
        return None


class HoverState(State):
    def tick(self, ctx: MissionContext) -> Optional[Type[State]]:
        ctx.setpoints.send_trajectory_setpoint(ctx.target_position_enu, ctx.target_yaw_deg)
        if ctx.hover_deadline is None:
            return TransitState
        if ctx.now() >= ctx.hover_deadline:
            if ctx.advance_waypoint():
                next_target = ctx.current_ned_target()
                ctx.logger.info(
                    "FSM state -> TRANSIT, wp_idx=%d, target=(%.2f, %.2f, %.2f)"
                    % (ctx.current_index, next_target[0], next_target[1], next_target[2])
                )
                return TransitState
            ctx.logger.info("FSM state -> HOLD, mission complete. Holding last setpoint (hover)")
            if ctx.plan.land_on_finish:
                ctx.queue_action("return_home")
                ctx.queue_action("land")
            return HoldState
        return None


class InspectState(State):
    def __init__(self) -> None:
        self._deadline: Optional[float] = None
        self._wait_for_ack = False

    def on_enter(self, ctx: MissionContext) -> None:
        self._wait_for_ack = ctx.plan.inspection.require_ack
        timeout = ctx.plan.inspection.timeout_s
        if self._wait_for_ack:
            self._deadline = ctx.now() + timeout
            ctx.logger.info("FSM state -> INSPECT (timeout %.1fs)" % timeout)
        else:
            self._deadline = None
            ctx.logger.info("FSM state -> INSPECT (hover-only)")

    def tick(self, ctx: MissionContext) -> Optional[Type[State]]:
        ctx.setpoints.send_trajectory_setpoint(ctx.target_position_enu, ctx.target_yaw_deg)
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

    def _advance_on_hover(self, ctx: MissionContext) -> Optional[Type[State]]:
        if ctx.hover_deadline is None:
            return TransitState
        if ctx.now() >= ctx.hover_deadline:
            return self._advance(ctx)
        return None

    def _advance(self, ctx: MissionContext) -> Optional[Type[State]]:
        if ctx.advance_waypoint():
            next_target = ctx.current_ned_target()
            ctx.logger.info(
                "FSM state -> TRANSIT, wp_idx=%d, target=(%.2f, %.2f, %.2f)"
                % (ctx.current_index, next_target[0], next_target[1], next_target[2])
            )
            return TransitState
        ctx.logger.info("FSM state -> HOLD, mission complete. Holding last setpoint (hover)")
        if ctx.plan.land_on_finish:
            ctx.enqueue_fallback("land")
        return HoldState


class HoldState(State):
    def tick(self, ctx: MissionContext) -> Optional[Type[State]]:
        ctx.setpoints.send_trajectory_setpoint(ctx.target_position_enu, ctx.target_yaw_deg)
        return None


class FallbackHoldState(State):
    def tick(self, ctx: MissionContext) -> Optional[Type[State]]:
        ctx.setpoints.send_trajectory_setpoint(ctx.target_position_enu, ctx.target_yaw_deg)
        if ctx.hold_until is None:
            ctx.complete_active_action()
            if not ctx.has_pending_actions():
                return HoldState
            return None
        if ctx.now() >= ctx.hold_until:
            ctx.logger.info("Fallback hold complete")
            ctx.complete_active_action()
            if not ctx.has_pending_actions():
                return HoldState
        return None


class ReturnHomeState(State):
    def tick(self, ctx: MissionContext) -> Optional[Type[State]]:
        target_enu = ctx.home_position_enu()
        ctx.setpoints.send_trajectory_setpoint(target_enu, 0.0)
        local = ctx.telemetry.local_position()
        if local is None:
            return None
        ned_target, _, _ = ctx.setpoints.describe_target(target_enu, 0.0)
        dx = ned_target[0] - local.x
        dy = ned_target[1] - local.y
        dz = ned_target[2] - local.z
        distance_xy = (dx ** 2 + dy ** 2) ** 0.5
        altitude_error = abs(dz)
        if distance_xy < ctx.tolerance_xy() and altitude_error < ctx.tolerance_z():
            ctx.logger.info("Return-home reached. Holding position")
            ctx.complete_active_action()
            if not ctx.has_pending_actions():
                return HoldState
        return None


class EmergencyLandState(State):
    def __init__(self) -> None:
        self._command_sent = False

    def on_enter(self, ctx: MissionContext) -> None:
        ctx.logger.error("Fallback engaged: EMERGENCY LAND")
        ctx.setpoints.send_vehicle_command(VEHICLE_CMD_NAV_LAND)
        self._command_sent = True
        ctx.complete_active_action()

    def tick(self, ctx: MissionContext) -> Optional[Type[State]]:
        ctx.setpoints.send_trajectory_setpoint(ctx.target_position_enu, ctx.target_yaw_deg)
        return None


class MissionStateMachine:
    def __init__(self, ctx: MissionContext) -> None:
        self._ctx = ctx
        self._state: State = BootstrapState()
        self._state.on_enter(ctx)
        ctx.publish_state(self._state.__class__.__name__)

    def tick(self) -> None:
        ctx = self._ctx
        self._process_external_events()
        next_state_cls = self._state.tick(ctx)
        if next_state_cls is not None:
            self._handle_step_action_transition(ctx, next_state_cls)
            self._transition(next_state_cls)
            return
        fallback_state = ctx.next_fallback_state()
        if fallback_state is not None and not isinstance(self._state, fallback_state):
            self._transition(fallback_state)

    def _handle_step_action_transition(self, ctx: MissionContext, next_state_cls: Type[State]) -> None:
        if isinstance(self._state, (HoverState, InspectState)) and ctx.current_step.action:
            action = ctx.current_step.action
            ctx.logger.info(f"Mission action trigger: {action} at step {ctx.current_index}")
            ctx.publish_state(action)

    def _process_external_events(self) -> None:
        ctx = self._ctx
        telem = ctx.telemetry
        if telem.battery_critical():
            ctx.enqueue_fallback("battery_critical")
        elif telem.battery_warning():
            ctx.enqueue_fallback("battery_warning")
        if not telem.data_link_ok():
            ctx.enqueue_fallback("link_lost")
        if telem.consume_low_light_event():
            ctx.enqueue_fallback("low_light")

        fallback_state = ctx.next_fallback_state()
        if fallback_state is not None and not isinstance(self._state, fallback_state):
            self._transition(fallback_state)

    def _transition(self, state_cls: Type[State]) -> None:
        self._state.on_exit(self._ctx)
        self._state = state_cls()
        self._state.on_enter(self._ctx)
        self._ctx.publish_state(self._state.__class__.__name__)


__all__ = [
    "MissionRuntime",
    "MissionContext",
    "MissionStateMachine",
    "LocalPosition",
]
