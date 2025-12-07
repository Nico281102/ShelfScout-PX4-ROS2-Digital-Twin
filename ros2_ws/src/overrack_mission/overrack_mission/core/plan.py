"""Mission plan parser supporting the v1 declarative mission language."""

from __future__ import annotations

import math
import pathlib
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

import yaml

from .bounds import EnuBounds
from .planning import Route, load_route

Point2D = Tuple[float, float]


class MissionPlanError(RuntimeError):
    """Raised when the mission file is invalid."""


@dataclass
class FallbackAction:
    name: str
    value: Optional[float] = None


@dataclass
class InspectionConfig:
    enable: bool = False
    timeout_s: float = 3.0
    require_ack: bool = False


@dataclass
class MissionStep:
    position: Tuple[float, float, float]
    yaw_deg: float = 0.0
    hover_override_s: Optional[float] = None
    inspect: bool = False
    action: Optional[str] = None

    @property
    def yaw_rad(self) -> float:
        return math.radians(self.yaw_deg)


@dataclass
class MissionPlan:
    altitude_m: float
    hover_time_s: float
    steps: List[MissionStep]
    inspection: InspectionConfig = field(default_factory=InspectionConfig)
    fallback: Dict[str, List[FallbackAction]] = field(default_factory=dict)
    land_on_finish: bool = False
    raw: Dict[str, object] = field(default_factory=dict)

    @property
    def home_position(self) -> Point2D:
        if self.steps:
            return (self.steps[0].position[0], self.steps[0].position[1])
        home = self.raw.get("home", (0.0, 0.0))
        try:
            return (float(home[0]), float(home[1]))
        except (TypeError, ValueError, IndexError):
            return (0.0, 0.0)

    @property
    def waypoints(self) -> List[Point2D]:
        return [(step.position[0], step.position[1]) for step in self.steps]


def load_plan(path: pathlib.Path) -> MissionPlan:
    data = yaml.safe_load(path.read_text()) or {}
    version = data.get("api_version")
    if version != 1:
        raise MissionPlanError("Mission 'api_version' must be 1")

    defaults = data.get("defaults", {})
    if not isinstance(defaults, dict):
        raise MissionPlanError("Mission 'defaults' must be a mapping")

    inspection_cfg = _parse_inspection(data.get("inspection"))

    mode = data.get("mode")
    if mode not in (None, "precomputed"):
        raise MissionPlanError("Only precomputed missions are supported; remove 'mode' or set it to 'precomputed'")

    route_file = data.get("route_file")
    if not isinstance(route_file, str) or not route_file:
        raise MissionPlanError("Mission must specify 'route_file'")

    route_path = _resolve_path(path, route_file)
    route = load_route(route_path)
    steps = _steps_from_route(route, inspection_cfg)
    altitude = float(route.default_altitude_m)
    hover_time = float(route.default_hover_s)

    fallback_cfg = _parse_fallbacks(data.get("fallback"))

    if "land_on_finish" in data and "return_home_and_land_on_finish" not in data:
        raise MissionPlanError("Use 'return_home_and_land_on_finish' instead of legacy 'land_on_finish'")
    land_on_finish = bool(data.get("return_home_and_land_on_finish", False))

    plan = MissionPlan(
        altitude_m=altitude,
        hover_time_s=hover_time,
        steps=steps,
        inspection=inspection_cfg,
        fallback=fallback_cfg,
        land_on_finish=land_on_finish,
        raw=data,
    )

    return plan


def _parse_inspection(value: object) -> InspectionConfig:
    if value is None:
        return InspectionConfig(enable=False, timeout_s=3.0, require_ack=False)
    if not isinstance(value, dict):
        raise MissionPlanError("Mission 'inspection' must be a mapping if provided")
    enable = bool(value.get("enable", False))
    timeout = _coerce_float(value, "timeout_s", 3.0)
    require_ack = bool(value.get("require_ack", False))
    return InspectionConfig(enable=enable, timeout_s=timeout, require_ack=require_ack)


def _coerce_float(container: Dict[str, object], key: str, default: float) -> float:
    value = container.get(key, default)
    try:
        return float(value)
    except (TypeError, ValueError) as exc:
        raise MissionPlanError(f"Mission parameter '{key}' must be a number") from exc


def _parse_fallbacks(value: object) -> Dict[str, List[FallbackAction]]:
    if value is None:
        return {}
    if not isinstance(value, dict):
        raise MissionPlanError("Mission 'fallback' must be a mapping")
    parsed: Dict[str, List[FallbackAction]] = {}
    for trigger, actions in value.items():
        if isinstance(actions, str):
            actions = [actions]
        if not isinstance(actions, Sequence) or not actions:
            raise MissionPlanError(f"Fallback actions for '{trigger}' must be a non-empty list")
        parsed[trigger] = [_parse_action(str(action)) for action in actions]
    return parsed


def _parse_action(value: str) -> FallbackAction:
    value = value.strip()
    if not value:
        raise MissionPlanError("Fallback action entries cannot be empty")
    if ":" not in value:
        return FallbackAction(name=value)
    name, raw_arg = value.split(":", 1)
    name = name.strip()
    raw_arg = raw_arg.strip().lower()
    seconds: Optional[float] = None
    if raw_arg.endswith("s"):
        raw_arg = raw_arg[:-1]
    if raw_arg:
        try:
            seconds = float(raw_arg)
        except ValueError as exc:
            raise MissionPlanError(f"Fallback action argument '{raw_arg}' is not a number") from exc
    return FallbackAction(name=name, value=seconds)


def validate_waypoints_in_bounds(plan: MissionPlan, bounds: EnuBounds) -> None:
    """Ensure every waypoint stays within the provided ENU bounds."""

    for idx, step in enumerate(plan.steps):
        x, y, z = step.position
        if not bounds.x.contains(x):
            raise MissionPlanError(
                f"Waypoint #{idx} x={x:.2f} exceeds bounds [{bounds.x.minimum}, {bounds.x.maximum}] (ENU)"
            )
        if not bounds.y.contains(y):
            raise MissionPlanError(
                f"Waypoint #{idx} y={y:.2f} exceeds bounds [{bounds.y.minimum}, {bounds.y.maximum}] (ENU)"
            )
        if not bounds.z.contains(z):
            raise MissionPlanError(
                f"Waypoint #{idx} z={z:.2f} exceeds bounds [{bounds.z.minimum}, {bounds.z.maximum}] (ENU)"
            )


def _resolve_path(base: pathlib.Path, value: str) -> pathlib.Path:
    candidate = pathlib.Path(value)
    if not candidate.is_absolute():
        candidate = (base.parent / candidate).resolve()
    if not candidate.exists():
        raise MissionPlanError(f"Route file not found: {candidate}")
    return candidate


def _steps_from_route(route: Route, inspection_cfg: InspectionConfig) -> List[MissionStep]:
    steps: List[MissionStep] = []
    for idx, step in enumerate(route.steps):
        position = _normalize_position(step.position, route.default_altitude_m, idx)
        yaw_deg = float(step.yaw_deg)
        hover_override = step.hover_s
        inspect = bool(step.inspect or inspection_cfg.enable)
        steps.append(MissionStep(position, yaw_deg, hover_override, inspect, step.action))
    return steps


def _normalize_position(position: Sequence[float], default_altitude: float, idx: int) -> Tuple[float, float, float]:
    if len(position) == 2:
        return (float(position[0]), float(position[1]), float(default_altitude))
    if len(position) >= 3:
        try:
            return (float(position[0]), float(position[1]), float(position[2]))
        except (TypeError, ValueError) as exc:
            raise MissionPlanError(f"Route step #{idx} has invalid position values") from exc
    raise MissionPlanError(f"Route step #{idx} position must have at least 2 values")
