"""Small helpers to compute mission KPIs in a ROS-free way."""

from __future__ import annotations

import csv
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path


@dataclass(frozen=True)
class MissionStats:
    mission_time_s: float
    inspections_total: int
    inspections_suspect: int
    defective_rate: float
    snapshots_total: int
    snapshots_low_light: int


def mission_duration(start: float | None, end: float | None) -> float:
    """Return mission duration (s) guarding against missing timestamps."""

    if start is None or end is None:
        return 0.0
    return max(0.0, end - start)


def defective_rate(total: int, suspect: int) -> float:
    if total <= 0:
        return 0.0
    return max(0.0, min(1.0, suspect / float(total)))


def export_csv(rows: list[tuple[str, object]], directory: Path, stem: str) -> Path:
    directory.mkdir(parents=True, exist_ok=True)
    timestamp = datetime.utcnow().strftime("%Y%m%d_%H%M%S")
    output = directory / f"{stem}_{timestamp}.csv"
    with output.open("w", newline="") as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(["metric", "value"])
        for key, value in rows:
            writer.writerow([key, value])
    return output


def export_inspection_log(
    entries: list[tuple[int, float, str]],
    directory: Path,
    stem: str,
    start_time: float | None,
) -> Path | None:
    if not entries:
        return None
    directory.mkdir(parents=True, exist_ok=True)
    timestamp = datetime.utcnow().strftime("%Y%m%d_%H%M%S")
    output = directory / f"{stem}_{timestamp}.csv"
    with output.open("w", newline="") as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(["snapshot_index", "seconds_from_start", "result"])
        t0 = start_time if start_time is not None else entries[0][1]
        for idx, stamp, result in entries:
            writer.writerow([idx, f"{max(0.0, stamp - t0):.2f}", result])
    return output


__all__ = [
    "MissionStats",
    "mission_duration",
    "defective_rate",
    "export_csv",
    "export_inspection_log",
]
