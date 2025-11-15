"""Common bounds helpers shared across core logic and PX4 adapters."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class AxisBounds:
    minimum: float
    maximum: float

    def contains(self, value: float) -> bool:
        return self.minimum <= value <= self.maximum

    def clamp(self, value: float) -> float:
        if value < self.minimum:
            return self.minimum
        if value > self.maximum:
            return self.maximum
        return value


@dataclass(frozen=True)
class NedBounds:
    """Bounds expressed in PX4's NED frame (meters)."""

    x: AxisBounds  # North
    y: AxisBounds  # East
    z: AxisBounds  # Down (negative above takeoff plane)

    def clamp(self, position: tuple[float, float, float]) -> tuple[float, float, float]:
        return (
            self.x.clamp(position[0]),
            self.y.clamp(position[1]),
            self.z.clamp(position[2]),
        )


@dataclass(frozen=True)
class EnuBounds:
    """Bounds expressed in the ENU frame used by mission planning."""

    x: AxisBounds  # East
    y: AxisBounds  # North
    z: AxisBounds  # Up (positive)


def enu_bounds_from_ned(bounds: NedBounds) -> EnuBounds:
    """Mirror NED bounds into ENU (swap axes, flip Z)."""

    # In PX4: x -> North, y -> East, z -> Down. ENU swaps the horizontal axes and flips Z.
    x_bounds = AxisBounds(bounds.y.minimum, bounds.y.maximum)
    y_bounds = AxisBounds(bounds.x.minimum, bounds.x.maximum)
    z_bounds = AxisBounds(-bounds.z.maximum, -bounds.z.minimum)
    return EnuBounds(x=x_bounds, y=y_bounds, z=z_bounds)


__all__ = [
    "AxisBounds",
    "NedBounds",
    "EnuBounds",
    "enu_bounds_from_ned",
]
