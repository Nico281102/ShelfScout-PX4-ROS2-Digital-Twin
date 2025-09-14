"""Simple barcode vision pipeline for the down-facing camera frames.

Watches a directory for new images (saved by the Gazebo camera) and tries to
decode barcodes using pyzbar (or zxing-cpp if available). Logs the detections
to CSV and optionally annotates/outputs preview images.

Optionally, if --with-mavsdk is passed, the script connects to PX4 SITL and
adds the current mission item index to each log row for correlation.
"""

import argparse
import csv
import time
from pathlib import Path
from typing import Optional, List

import cv2

try:
    from pyzbar import pyzbar  # type: ignore
except Exception:  # pragma: no cover
    pyzbar = None

try:
    import zxingcpp  # type: ignore
except Exception:  # pragma: no cover
    zxingcpp = None


async def _mission_index() -> int:
    """Yield current mission index via MAVSDK if available, else -1.

    Imported lazily to avoid requiring mavsdk when not requested.
    """
    try:
        from mavsdk import System  # type: ignore
    except Exception:
        return -1

    idx = -1
    drone = System()
    await drone.connect(system_address="udp://:14540")
    async for _state in drone.core.connection_state():
        break
    async for p in drone.mission.mission_progress():
        idx = p.current
        return idx
    return idx


def decode_barcodes(img) -> List[str]:
    codes: List[str] = []
    if pyzbar is not None:
        for b in pyzbar.decode(img):
            try:
                codes.append(b.data.decode("utf-8"))
            except Exception:
                pass
    if not codes and zxingcpp is not None:
        try:
            res = zxingcpp.read_barcodes(img)
            for r in res:
                if r.text:
                    codes.append(r.text)
        except Exception:
            pass
    return codes


def run_pipeline(watch_dir: Path, out_csv: Path, preview_dir: Optional[Path] = None,
                 with_mavsdk: bool = False) -> None:
    watch_dir.mkdir(parents=True, exist_ok=True)
    out_csv.parent.mkdir(parents=True, exist_ok=True)
    if preview_dir:
        preview_dir.mkdir(parents=True, exist_ok=True)

    seen: set[str] = set()
    last_size = -1
    with out_csv.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(["t", "file", "codes", "mission_idx"])  # header

        print(f"[Vision] Watching: {watch_dir}  -> CSV: {out_csv}")
        while True:
            # Process newest file first
            files = sorted([p for p in watch_dir.glob("*.png")] + [p for p in watch_dir.glob("*.jpg")])
            if not files:
                time.sleep(0.2)
                continue
            latest = files[-1]

            # Avoid reprocessing exact same file
            stat = latest.stat()
            sig = f"{latest.name}:{stat.st_mtime_ns}:{stat.st_size}"
            if sig == last_size:
                time.sleep(0.2)
                continue
            last_size = sig

            img = cv2.imread(str(latest))
            if img is None:
                time.sleep(0.1)
                continue

            codes = decode_barcodes(img)
            if codes:
                mission_idx = -1
                if with_mavsdk:
                    try:
                        import asyncio
                        mission_idx = asyncio.run(_mission_index())
                    except Exception:
                        mission_idx = -1

                writer.writerow([int(time.time()*1000), latest.name, ";".join(codes), mission_idx])
                f.flush()

                if preview_dir:
                    # Draw simple overlay with codes
                    overlay = img.copy()
                    y = 30
                    for c in codes:
                        cv2.putText(overlay, c, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                        y += 30
                    cv2.imwrite(str(preview_dir / latest.name), overlay)

            time.sleep(0.2)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--watch", type=Path, default=Path("data/images/downcam"), help="Directory with camera frames")
    ap.add_argument("--csv", type=Path, default=Path("data/logs/vision.csv"), help="Output CSV path")
    ap.add_argument("--preview", type=Path, default=None, help="Optional preview frames output dir")
    ap.add_argument("--with-mavsdk", action="store_true", help="Attach mission index using MAVSDK")
    args = ap.parse_args()

    run_pipeline(args.watch, args.csv, args.preview, args.with_mavsdk)


if __name__ == "__main__":
    main()
