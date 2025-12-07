"""Utilities for loading precomputed routes from YAML files."""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Optional, Sequence

import yaml


@dataclass
class RouteStep:
    name: str
    position: Sequence[float]
    yaw_deg: float = 0.0
    hover_s: Optional[float] = None
    inspect: bool = False
    action: Optional[str] = None

    def __post_init__(self) -> None:
        if len(self.position) not in (2, 3):
            raise ValueError(f"Route step '{self.name}' position must have 2 or 3 values")


@dataclass
class Route:
    name: str
    steps: List[RouteStep] = field(default_factory=list)
    default_altitude_m: float = 2.5
    default_hover_s: float = 2.0


class RouteFormatError(RuntimeError):
    """Raised when the route YAML is invalid."""


def load_route(path: Path) -> Route:
    data = yaml.safe_load(path.read_text())
    if not isinstance(data, dict) or "route" not in data:
        raise RouteFormatError("Route file must contain a top-level 'route' mapping")

    route_raw = data["route"]
    if not isinstance(route_raw, dict):
        raise RouteFormatError("'route' must be a mapping")

    name = str(route_raw.get("name", path.stem))
    default_altitude = float(route_raw.get("default_altitude_m", 2.5))
    default_hover = float(route_raw.get("default_hover_s", 2.0))

    steps_raw = route_raw.get("steps")
    if not isinstance(steps_raw, list) or not steps_raw:
        raise RouteFormatError("Route must define a non-empty 'steps' list")

    steps: List[RouteStep] = []
    for idx, step_raw in enumerate(steps_raw):
        if not isinstance(step_raw, dict):
            raise RouteFormatError(f"Route step #{idx} must be a mapping")
        step_name = str(step_raw.get("name", f"step_{idx}"))
        position = step_raw.get("position")
        if not isinstance(position, (list, tuple)):
            raise RouteFormatError(f"Route step '{step_name}' missing 'position'")
        try:
            position = [float(v) for v in position]
        except (TypeError, ValueError) as exc:
            raise RouteFormatError(f"Route step '{step_name}' has invalid position values") from exc
        yaw_deg = float(step_raw.get("yaw_deg", 0.0))
        hover_s = step_raw.get("hover_s")
        if hover_s is not None:
            hover_s = float(hover_s)
        inspect = bool(step_raw.get("inspect", False))
        action = step_raw.get("action")
        if isinstance(action, str) and action.strip().lower() == "land":
            raise RouteFormatError(
                f"Route step '{step_name}' uses unsupported action 'land'; "
                "use fallback actions or return_home_and_land_on_finish instead."
            )
        steps.append(RouteStep(step_name, position, yaw_deg, hover_s, inspect, action))

    return Route(
        name=name,
        steps=steps,
        default_altitude_m=default_altitude,
        default_hover_s=default_hover,
    )


__all__ = ["Route", "RouteStep", "RouteFormatError", "load_route"]
