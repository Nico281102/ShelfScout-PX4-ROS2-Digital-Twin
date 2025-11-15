"""Coverage planning utilities (e.g. lawnmower / boustrophedon paths)."""

from __future__ import annotations

import math
from typing import Iterable, List, Sequence, Tuple

Point2D = Tuple[float, float]


def lawnmower_path(
    polygon: Sequence[Sequence[float]],
    altitude_m: float,
    fov_deg: float,
    overlap: float,
    *,
    start_from_left: bool = True,
) -> List[Point2D]:
    """Generate a simple serpentine path covering a polygon's bounding box.

    The implementation approximates coverage by sweeping the bounding box of the
    polygon with parallel lines spaced by the camera footprint (adjusted for the
    desired overlap). For indoor demos this provides a quick approximation and can
    be swapped with a richer planner later.
    """

    if altitude_m <= 0:
        raise ValueError("altitude_m must be positive")
    if fov_deg <= 0 or fov_deg >= 180:
        raise ValueError("fov_deg must be between 0 and 180")
    if not polygon:
        raise ValueError("polygon must contain at least one vertex")

    xs, ys = zip(*((float(p[0]), float(p[1])) for p in polygon))
    xmin, xmax = min(xs), max(xs)
    ymin, ymax = min(ys), max(ys)
    if xmax == xmin or ymax == ymin:
        raise ValueError("polygon must span a non-zero area")

    swath = 2.0 * altitude_m * math.tan(math.radians(fov_deg) / 2.0)
    effective_overlap = min(max(overlap, 0.0), 0.95)
    step = max(0.1, swath * (1.0 - effective_overlap))

    y = ymin + step / 2.0
    path: List[Point2D] = []
    sweep_left_to_right = start_from_left

    while y <= ymax:
        if sweep_left_to_right:
            path.extend([(xmin, y), (xmax, y)])
        else:
            path.extend([(xmax, y), (xmin, y)])
        sweep_left_to_right = not sweep_left_to_right
        y += step

    if not path:
        # fallback: visit the polygon vertices in order
        path = [(float(p[0]), float(p[1])) for p in polygon]

    return path
