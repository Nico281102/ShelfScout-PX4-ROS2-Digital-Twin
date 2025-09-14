"""Mission bridge: upload waypoints to PX4 SITL via MAVSDK.

Reads waypoints from config/mission.yaml (local XY in meters) and converts them
to global coordinates relative to the vehicle home. Then uploads a mission and
starts it.

Usage:
  python3 scripts/mission_bridge.py --start        # upload + start mission
  python3 scripts/mission_bridge.py --upload-only  # only upload

Requirements:
  - PX4 SITL running and reachable at udp://:14540
  - mavsdk-python installed (see requirements.txt)
"""

import asyncio
import math
import argparse
import time
from pathlib import Path
import sys
import os

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from src.config import load_config  # type: ignore

from mavsdk import System
from mavsdk.mission import MissionItem, MissionPlan


def meters_to_deg(lat_deg: float, north_m: float, east_m: float) -> tuple[float, float]:
    """Convert local N/E meters to lat/lon degree offsets at a given latitude.

    Assumes small offsets; sufficient for indoor sim grid.
    """
    meters_per_deg_lat = 111_111.0
    meters_per_deg_lon = 111_111.0 * math.cos(math.radians(lat_deg))
    dlat = north_m / meters_per_deg_lat
    dlon = east_m / meters_per_deg_lon
    return dlat, dlon


async def make_system(deadline_secs: int = 60) -> System:
    """Connect to PX4 using MAVSDK, with URL fallback and retry until deadline.

    Attempt order per try:
      1) MAVSDK_URL (if set)
      2) udpin://0.0.0.0:14540
      3) udpin://:14540
      4) udp://0.0.0.0:14540
      5) udp://:14540
    """
    start = asyncio.get_event_loop().time()
    attempt = 0
    last_err: Exception | None = None
    while (asyncio.get_event_loop().time() - start) < deadline_secs:
        attempt += 1
        candidates: list[str] = []
        env_url = os.getenv("MAVSDK_URL")
        if env_url:
            candidates.append(env_url)
        candidates += [
            "udpin://0.0.0.0:14540",
            "udpin://:14540",
            "udp://0.0.0.0:14540",
            "udp://:14540",
            "tcp://127.0.0.1:4560",
        ]

        for url in candidates:
            drone = System()
            try:
                print(f"[MissionBridge] Attempt {attempt}: trying {url} ...")
                await drone.connect(system_address=url)
                # Wait briefly for connection
                async def _wait_conn():
                    async for state in drone.core.connection_state():
                        if state.is_connected:
                            return True
                await asyncio.wait_for(_wait_conn(), timeout=6.0)
                print(f"[MissionBridge] Connected via {url}")
                # Touch home to ensure telemetry is flowing
                async for _hp in drone.telemetry.home():
                    break
                return drone
            except Exception as e:
                last_err = e
                print(f"[MissionBridge] Failed {url}: {e}")
                continue
        # Backoff before next attempt round
        await asyncio.sleep(min(5, 1 + attempt))
    raise RuntimeError(f"Unable to connect to PX4 via MAVSDK within {deadline_secs}s. Last error: {last_err}")


async def wait_preflight_ready(
    drone: System,
    timeout_s: float = 90.0,
    require_global: bool = True,
    require_local: bool = True,
    require_home: bool = True,
) -> None:
    """Wait until telemetry health indicates EKF/global/local/home ready.

    If a flag is not present on the Health message, it is ignored.
    """
    print("[Preflight] Waiting for EKF/GPS/Home ...")
    t0 = time.monotonic()
    last_print = 0.0
    async for h in drone.telemetry.health():
        def _has(attr: str) -> bool:
            return hasattr(h, attr)

        ok = True
        if require_global and _has("is_global_position_ok"):
            ok &= bool(getattr(h, "is_global_position_ok"))
        if require_local and _has("is_local_position_ok"):
            ok &= bool(getattr(h, "is_local_position_ok"))
        if require_home and _has("is_home_position_ok"):
            ok &= bool(getattr(h, "is_home_position_ok"))

        now = time.monotonic()
        if now - last_print > 2.5:
            # Periodic snapshot
            gp = getattr(h, "is_global_position_ok", None)
            lp = getattr(h, "is_local_position_ok", None)
            hp = getattr(h, "is_home_position_ok", None)
            print(f"[Preflight] health: global={gp} local={lp} home={hp}")
            last_print = now

        if ok:
            print("[Preflight] Health OK (global/local/home).")
            break

        if now - t0 > timeout_s:
            raise TimeoutError(
                f"Preflight not ready within {timeout_s}s: "
                f"global={getattr(h,'is_global_position_ok',None)} "
                f"local={getattr(h,'is_local_position_ok',None)} "
                f"home={getattr(h,'is_home_position_ok',None)}"
            )

    await asyncio.sleep(0.5)


async def upload_and_start(start: bool, cfg_path: Path, arm_wo_gps: bool = False) -> None:
    cfg = load_config(cfg_path)
    drone = await make_system()

    # Optional: relax arming in indoor runs
    if arm_wo_gps:
        try:
            print("[Preflight] --arm-wo-gps set: COM_ARM_WO_GPS=1, NAV_DLL_ACT=0")
            await drone.param.set_param_int("COM_ARM_WO_GPS", 1)
            await drone.param.set_param_int("NAV_DLL_ACT", 0)
        except Exception as e:
            print(f"[Preflight] Warning: failed to set params for arm-without-GPS: {e}")

    # Get home location
    home = None
    async for hp in drone.telemetry.home():
        home = hp
        break
    if home is None:
        raise RuntimeError("Home position not available from telemetry")

    # Build mission items
    items: list[MissionItem] = []
    for i, (x, y) in enumerate(cfg["waypoints"]):
        # Our config uses X=East, Y=North (meters)
        east_m = float(x)
        north_m = float(y)
        dlat, dlon = meters_to_deg(home.latitude_deg, north_m, east_m)
        lat = home.latitude_deg + dlat
        lon = home.longitude_deg + dlon
        alt = cfg["altitude_m"]

        # MissionItem API compatibility shim (handles newer 'vehicle_action' param)
        def make_item() -> MissionItem:
            speed = cfg["cruise_speed_mps"]
            try:
                return MissionItem(
                    lat, lon, alt,
                    speed,
                    True,
                    float("nan"),
                    float("nan"),
                    MissionItem.CameraAction.NONE,
                    0.0,
                    float("nan"),
                    1.0,
                    float("nan"),
                    float("nan"),
                    MissionItem.VehicleAction.NONE,  # type: ignore[attr-defined]
                )
            except TypeError:
                try:
                    return MissionItem(
                        lat, lon, alt,
                        speed,
                        True,
                        float("nan"),
                        float("nan"),
                        MissionItem.CameraAction.NONE,
                        0.0,
                        float("nan"),
                        1.0,
                        float("nan"),
                        float("nan"),
                    )
                except TypeError:
                    return MissionItem(
                        lat, lon, alt,
                        speed,
                        True,
                        float("nan"),
                        float("nan"),
                        MissionItem.CameraAction.NONE,
                        0.0,
                        float("nan"),
                    )

        item = make_item()
        items.append(item)

    plan = MissionPlan(items)
    await drone.mission.clear_mission()
    await drone.mission.upload_mission(plan)
    print(f"[Mission] Uploaded {len(items)} waypoints from {cfg_path}")

    if start:
        # Wait for EKF/preflight readiness before arming
        try:
            await wait_preflight_ready(
                drone,
                timeout_s=90.0,
                require_global=not arm_wo_gps,
                require_local=True,
                require_home=True,
            )
        except TimeoutError as te:
            print(f"[Preflight] Timeout: {te}")
            print("[Mission] Aborting due to preflight readiness timeout.")
            return

        # Ensure mission auto RTL after completion
        try:
            await drone.mission.set_return_to_launch_after_mission(True)
        except Exception:
            pass

        print("[Mission] Arming and starting mission...")
        await drone.action.arm()
        await drone.mission.start_mission()
        # Optional: wait for mission completion
        async for m in drone.mission.mission_progress():
            print(f"  Progress: {m.current}/{m.total}")
            if m.current == m.total:
                break
        print("[Mission] Completed.")


async def _main():
    parser = argparse.ArgumentParser()
    repo_root = ROOT
    default_cfg = repo_root / "config" / "mission.yaml"
    parser.add_argument("--config", type=Path, default=default_cfg, help="Path to mission YAML (defaults to repo config/mission.yaml)")
    parser.add_argument("--upload-only", action="store_true", help="Only upload mission, do not start")
    parser.add_argument("--start", action="store_true", help="Upload and start mission")
    parser.add_argument(
        "--arm-wo-gps",
        action="store_true",
        help="Allow arming without GPS (sets COM_ARM_WO_GPS=1, NAV_DLL_ACT=0 and relaxes global position requirement)",
    )
    args = parser.parse_args()
    await upload_and_start(
        start=args.start and not args.upload_only,
        cfg_path=args.config,
        arm_wo_gps=args.arm_wo_gps,
    )


if __name__ == "__main__":
    asyncio.run(_main())
