"""Shared utilities to parse overrack_mission parameter YAML files.

This module centralizes the logic to read defaults (mission/world/agent) and
per-drone entries from the same YAML structure used by the launch scripts and
shell wrappers. It also exposes a small CLI for shell scripts to emit exported
variables without duplicating parsing logic in bash.
"""
from __future__ import annotations

import argparse
import json
import shlex
import sys
from pathlib import Path
from typing import Any, Dict, List

try:
    import yaml
except Exception as exc:  # pragma: no cover - handled at runtime
    sys.stderr.write(f"[param_utils] PyYAML is required: {exc}\n")
    sys.exit(1)


SUPPORTED_MULTI_MODELS = {
    "iris",
    "plane",
    "standard_vtol",
    "rover",
    "r1_rover",
    "typhoon_h480",
    "iris_opt_flow",
}


# ---------------------------------------------------------------------------
# Core helpers
# ---------------------------------------------------------------------------

def _nested(data: Dict[str, Any], keys: List[str]) -> str:
    ref: Any = data
    for key in keys:
        if not isinstance(ref, dict):
            return ""
        ref = ref.get(key)
    return ref if isinstance(ref, str) else ""


def load_config(path: Path) -> Dict[str, Any]:
    try:
        content = path.read_text(encoding="utf-8")
    except FileNotFoundError:
        return {}
    try:
        data = yaml.safe_load(content) or {}
        return data if isinstance(data, dict) else {}
    except Exception as exc:
        sys.stderr.write(f"[param_utils] Failed to parse {path}: {exc}\n")
        return {}


def drones_from_config(config: Dict[str, Any]) -> List[Dict[str, Any]]:
    ros_params = config.get("run_ros2_system", {}).get("ros__parameters", {})
    drones = ros_params.get("drones_yaml") or ""
    if isinstance(drones, str):
        try:
            drones = yaml.safe_load(drones) or []
        except Exception:
            drones = []
    if not isinstance(drones, list):
        return []
    return [d for d in drones if isinstance(d, dict)]


def default_mission(config: Dict[str, Any]) -> str:
    return _nested(config, ["run_ros2_system", "ros__parameters", "mission_file"]) or _nested(
        config, ["mission_runner", "ros__parameters", "mission_file"]
    )


def default_world(config: Dict[str, Any]) -> str:
    return _nested(config, ["run_ros2_system", "ros__parameters", "world_file"])


def default_agent_cmd(config: Dict[str, Any]) -> str:
    agent = _nested(config, ["run_ros2_system", "ros__parameters", "agent_cmd"])
    fallback = _nested(config, ["run_ros2_system", "ros__parameters", "agent_cmd_default"])
    return fallback or agent


def default_model(config: Dict[str, Any]) -> str:
    return _nested(config, ["mission_runner", "ros__parameters", "gazebo_model_name"])


def default_mavlink_url(config: Dict[str, Any]) -> str:
    return _nested(config, ["px4_param_setter", "ros__parameters", "mavlink_url"])


def section_params(config: Dict[str, Any], section: str) -> Dict[str, Any]:
    params = config.get(section, {}).get("ros__parameters", {})
    return params if isinstance(params, dict) else {}


def resolve_gazebo_model_name(drone: Dict[str, Any], idx: int, model_default: str) -> str:
    """Mirror sitl_multiple_run.sh naming so spawn offset detection matches Gazebo."""
    explicit = str(drone.get("gazebo_model_name") or "").strip()
    if explicit:
        return explicit

    base_model = str(drone.get("model") or model_default or "iris").strip()
    if not base_model:
        base_model = "iris"

    resolved_model = base_model if base_model in SUPPORTED_MULTI_MODELS else "iris"
    return f"{resolved_model}_{idx}"


# ---------------------------------------------------------------------------
# Shell emission helpers
# ---------------------------------------------------------------------------

def _quote(value: Any) -> str:
    return shlex.quote("" if value is None else str(value))


def emit_defaults_shell(config: Dict[str, Any]) -> str:
    drones = drones_from_config(config)
    lines = [
        f"YAML_MISSION_DEFAULT={_quote(default_mission(config))}",
        f"YAML_WORLD_DEFAULT={_quote(default_world(config))}",
        f"YAML_AGENT_DEFAULT={_quote(default_agent_cmd(config))}",
        f"YAML_DRONES_COUNT={len(drones)}",
    ]
    return "\n".join(lines)


def emit_drones_shell(config: Dict[str, Any]) -> str:
    drones = drones_from_config(config)
    lines = [f"DRONE_COUNT={len(drones)}"]
    for idx, drone in enumerate(drones):
        name = drone.get("name") or f"drone{idx+1}"
        ns = (drone.get("namespace") or name).lstrip("/")
        model = drone.get("model") or drone.get("gazebo_model_name") or "iris_opt_flow"
        spawn = drone.get("spawn") or {}
        mission = drone.get("mission_file") or ""
        mav_sys_id = drone.get("mav_sys_id") or ""
        mavlink_port = drone.get("mavlink_udp_port") or ""
        agent_cmd = drone.get("agent_cmd") or ""
        log_dir = drone.get("log_dir") or ""
        px4_params = drone.get("px4_params") if isinstance(drone.get("px4_params"), dict) else {}

        def emit(key: str, value: Any) -> None:
            lines.append(f"DRONE_{key}[{idx}]={_quote(value)}")

        emit("NAME", name)
        emit("NS", ns)
        emit("MODEL", model)
        emit("SPAWN_X", (spawn.get("x") if isinstance(spawn, dict) else 0.0) or 0.0)
        emit("SPAWN_Y", (spawn.get("y") if isinstance(spawn, dict) else 0.0) or 0.0)
        emit("SPAWN_Z", (spawn.get("z") if isinstance(spawn, dict) else 0.0) or 0.0)
        emit("SPAWN_YAW", (spawn.get("yaw") if isinstance(spawn, dict) else 0.0) or 0.0)
        emit("MISSION", mission)
        emit("MAV_SYS_ID", mav_sys_id)
        emit("MAVLINK_PORT", mavlink_port)
        emit("AGENT_CMD", agent_cmd)
        emit("LOG_DIR", log_dir)
        emit("PX4_PARAMS", json.dumps(px4_params))

    return "\n".join(lines)


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def _build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Parse overrack_mission params YAML")
    parser.add_argument("--file", required=True, help="Path to params YAML")
    subparsers = parser.add_subparsers(dest="command", required=True)

    defaults = subparsers.add_parser("defaults", help="Emit mission/world/agent defaults")
    defaults.add_argument("--format", choices=["shell", "json"], default="shell")

    drones = subparsers.add_parser("drones", help="Emit per-drone shell exports")
    drones.add_argument("--format", choices=["shell", "json"], default="shell")

    return parser


def main(argv: List[str] | None = None) -> int:
    args = _build_arg_parser().parse_args(argv)
    path = Path(args.file)
    config = load_config(path)

    if args.command == "defaults":
        if args.format == "json":
            payload = {
                "mission_file": default_mission(config),
                "world_file": default_world(config),
                "agent_cmd": default_agent_cmd(config),
                "drones_count": len(drones_from_config(config)),
            }
            print(json.dumps(payload))
        else:
            print(emit_defaults_shell(config))
        return 0

    if args.command == "drones":
        drones = drones_from_config(config)
        if args.format == "json":
            print(json.dumps(drones))
        else:
            print(emit_drones_shell(config))
        return 0

    return 1


if __name__ == "__main__":  # pragma: no cover
    sys.exit(main())
