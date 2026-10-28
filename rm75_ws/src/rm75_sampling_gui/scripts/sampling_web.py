#!/usr/bin/env python3
"""Start the RM75 FastAPI backend and serve the production React build."""
import argparse
import logging
import os
from pathlib import Path
import sys

import uvicorn

from rm75_sampling_gui.web.backend.app import create_app
from rm75_sampling_gui.web.backend.services import WebRuntime, load_web_configuration


PACKAGE = Path(__file__).resolve().parent.parent


def arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--offline", action="store_true")
    parser.add_argument("--simulation", choices=("true", "false"), default="false")
    parser.add_argument("--config", default=str(PACKAGE / "config/gui.yaml"))
    parser.add_argument("--tools", default=str(PACKAGE / "config/tools.yaml"))
    parser.add_argument("--frontend", default=str(PACKAGE / "web/frontend/dist"))
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8088)
    for option in ("execution-enabled", "qr-execution-authorized",
                   "joint-control-execution-authorized",
                   "cartesian-jog-execution-authorized",
                   "touch-z-force-authorized",
                   "tool5-drill-authorized",
                   "trajectory-execution-authorized",
                   "supervised-commissioning-authorized",
                   "sampling-hardware-authorized",
                   "unified-control-authorized"):
        parser.add_argument("--" + option, choices=("true", "false"), default="false")
    # roslaunch appends ROS remapping arguments; they are not Web options.
    return parser.parse_known_args()[0]


def main() -> int:
    args = arguments()
    if args.host not in ("127.0.0.1", "localhost"):
        raise SystemExit("默认Web平台无身份认证，仅允许绑定127.0.0.1")
    enabled = {name: getattr(args, name) == "true" for name in (
        "execution_enabled", "qr_execution_authorized",
        "joint_control_execution_authorized", "cartesian_jog_execution_authorized",
        "trajectory_execution_authorized",
        "touch_z_force_authorized",
        "tool5_drill_authorized",
        "supervised_commissioning_authorized", "sampling_hardware_authorized")}
    unified_control = args.unified_control_authorized == "true"
    simulation = args.simulation == "true"
    if args.offline and simulation:
        raise SystemExit("--offline与--simulation不能同时使用")
    if (args.offline or simulation) and (any(enabled.values()) or unified_control):
        raise SystemExit("OFFLINE/SIMULATION不能与真实执行授权同时使用")
    command_paths = sum(bool(enabled[key]) for key in (
        "joint_control_execution_authorized",
        "cartesian_jog_execution_authorized",
        "trajectory_execution_authorized",
        "touch_z_force_authorized",
        "tool5_drill_authorized",
    ))
    if command_paths > 1:
        raise SystemExit("七轴、trajectory bridge与Z轴力位遥操作只能授权一个")
    overrides = {
        "execution_enabled": enabled["execution_enabled"],
        "qr_execution_authorized": enabled["qr_execution_authorized"],
        "joint_control_execution_authorized": enabled["joint_control_execution_authorized"],
        "cartesian_jog_execution_authorized": enabled["cartesian_jog_execution_authorized"],
        "touch_z_force_authorized": enabled["touch_z_force_authorized"],
        "tool5_drill_authorized": enabled["tool5_drill_authorized"],
        "trajectory_execution_authorized": enabled["trajectory_execution_authorized"],
        "supervised_commissioning_authorized": enabled["supervised_commissioning_authorized"],
        "sampling_hardware_authorized": enabled["sampling_hardware_authorized"],
        "unified_control_authorized": unified_control,
    }
    config = load_web_configuration(args.config, args.tools, overrides)
    logging.basicConfig(level=getattr(logging, str(config.get("log_level", "INFO"))),
                        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s")
    runtime = WebRuntime(config, config["tools"], offline=args.offline,
                         simulation=simulation)
    app = create_app(runtime, args.frontend)
    uvicorn.run(app, host=args.host, port=args.port, log_level="info",
                access_log=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
