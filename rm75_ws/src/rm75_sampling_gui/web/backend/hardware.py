"""Bounded control for the RS-485 probes/pump and the DM-J4310 motor.

The low-level vendor files are staged by the unified ``start_sampling_web.sh``.
This module deliberately exposes a small allow-list instead of shell or serial
passthrough APIs.
"""
from __future__ import annotations

import importlib.util
import logging
import math
from pathlib import Path
import re
import subprocess
import sys
import threading
import time
from typing import Any, Callable, Dict, Optional


LOG = logging.getLogger("rm75_sampling_gui.web.hardware")
PUMP_CHANNEL = 4
PUMP_BIT = 1 << (PUMP_CHANNEL - 1)
MOTOR_MAX_SPEED_RAD_S = 1.0
MOTOR_MAX_DURATION_S = 60.0
MOTOR_MAX_CURRENT_A = 4.0
MOTOR_MAX_TORQUE_NM = 15.0


def _load_modbus(path: str) -> Any:
    source = Path(path)
    if not source.is_file():
        raise RuntimeError("RS-485底层文件不存在：{}".format(source))
    spec = importlib.util.spec_from_file_location("rm75_staged_modbus", str(source))
    if spec is None or spec.loader is None:
        raise RuntimeError("无法装载RS-485底层文件：{}".format(source))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class SamplingHardware:
    """Own the only RS-485 connection and serialize all motor processes."""

    def __init__(self, config: Dict[str, Any], authorized: bool = False,
                 simulation: bool = False) -> None:
        cfg = dict(config.get("sampling_hardware", {}))
        self.authorized = bool(authorized)
        self.simulation = bool(simulation)
        self.rs485_port = str(cfg.get("rs485_port", "/dev/rs485"))
        self.modbus_path = str(cfg.get(
            "modbus_path", "/root/rm75_ws/.rm75_hardware/modbus.py"))
        self.motor_script = str(cfg.get(
            "motor_script", "/root/rm75_ws/.rm75_hardware/motor_control.py"))
        self.motor_port = str(cfg.get(
            "motor_port", "/dev/serial/by-id/usb-HDSC_CDC_Device_00000000050C-if00"))
        self.poll_hz = max(0.2, min(5.0, float(cfg.get("poll_hz", 1.0))))
        drill_cfg = dict(config.get("tool5_drill", {}))
        self.motor_watchdog_timeout_s = float(
            drill_cfg.get("motor_watchdog_timeout_s", 1.0))
        if not 0.25 <= self.motor_watchdog_timeout_s <= 2.0:
            raise ValueError("电机连续运行看门狗必须在0.25到2秒之间")
        self._lock = threading.RLock()
        self._motor_lock = threading.Lock()
        self._motor_process: Optional[subprocess.Popen] = None
        self._simulation_motor_stop = threading.Event()
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._bus: Any = None
        self._modbus: Any = None
        self._sensor: Dict[str, Any] = {
            "online": False, "ec": None, "ec_temp": None, "orp": None,
            "ph": None, "relay_state": 0, "relay_online": False, "pump_on": False,
            "health": {}, "status": "硬件控制未授权", "updated": None,
        }
        self._motor: Dict[str, Any] = {
            "online": False, "busy": False, "command": "idle",
            "status": "硬件控制未授权", "updated": None,
            "control_mode": "VEL",
            "limits_verified": False,
            "max_speed": MOTOR_MAX_SPEED_RAD_S,
            "max_current": MOTOR_MAX_CURRENT_A,
            "max_torque": MOTOR_MAX_TORQUE_NM,
        }
        if self.simulation:
            self._sensor.update({"online": True, "relay_online": True,
                                 "health": {name: "ok" for name in
                                            ("EC", "ORP", "pH", "继电器")},
                                 "status": "SIMULATION"})
            self._motor.update({"online": True, "status": "SIMULATION",
                                "limits_verified": True})
        elif self.authorized:
            self._thread = threading.Thread(target=self._poll_loop,
                                            name="rm75-rs485", daemon=True)
            self._thread.start()

    @staticmethod
    def _bits_to_mask(bits: Any) -> int:
        return sum((1 << index) for index, enabled in enumerate(bits) if enabled)

    def _connect_bus(self) -> None:
        self._modbus = _load_modbus(self.modbus_path)
        bus = self._modbus.ModbusBus(self.rs485_port, 9600, 0.15, 2)
        bus.open()
        self._bus = bus
        LOG.info("采样RS-485已打开：%s", self.rs485_port)

    def _poll_once(self) -> None:
        if self._bus is None:
            self._connect_bus()
        values: Dict[str, Any] = {"updated": time.time()}
        health: Dict[str, str] = {}
        successes = 0
        try:
            ec_regs = self._bus.read_holding(1, 0, 2, "EC")
            values.update({"ec": float(ec_regs[0]),
                           "ec_temp": self._modbus.s16(ec_regs[1]) / 10.0})
            health["EC"] = "ok"
            successes += 1
        except Exception as error:
            health["EC"] = "ERR {}".format(error)
        try:
            values["orp"] = float(self._modbus.s16(
                self._bus.read_holding(2, 0, 1, "ORP")[0]))
            health["ORP"] = "ok"
            successes += 1
        except Exception as error:
            health["ORP"] = "ERR {}".format(error)
        try:
            values["ph"] = float(self._bus.read_holding(3, 0, 1, "pH")[0] / 100.0)
            health["pH"] = "ok"
            successes += 1
        except Exception as error:
            health["pH"] = "ERR {}".format(error)
        try:
            mask = self._bits_to_mask(self._bus.read_coils(4, 0, 8, "继电器状态"))
            values.update({"relay_state": mask, "relay_online": True,
                           "pump_on": bool(mask & PUMP_BIT)})
            health["继电器"] = "ok"
            successes += 1
        except Exception as error:
            values["relay_online"] = False
            health["继电器"] = "ERR {}".format(error)
        values["online"] = successes > 0
        values["health"] = health
        values["status"] = " ".join("{}={}".format(key, value)
                                      for key, value in health.items())
        values["status"] += " | " + self._bus.stats
        with self._lock:
            self._sensor.update(values)

    def _poll_loop(self) -> None:
        period = 1.0 / self.poll_hz
        while not self._stop.is_set():
            started = time.monotonic()
            try:
                self._poll_once()
            except Exception as error:  # hardware disconnect is reported, not fatal to HMI
                with self._lock:
                    self._sensor.update({"online": False, "status": str(error),
                                         "updated": time.time()})
                if self._bus is not None:
                    try:
                        self._bus.close()
                    except Exception:
                        pass
                    self._bus = None
            self._stop.wait(max(0.05, period - (time.monotonic() - started)))

    def snapshot(self) -> Dict[str, Any]:
        if self.simulation:
            phase = time.monotonic()
            with self._lock:
                self._sensor.update({
                    "ec": 1200.0 + 180.0 * math.sin(phase / 4.0),
                    "ec_temp": 18.0 + 2.0 * math.sin(phase / 11.0),
                    "ph": 7.0 + 0.3 * math.sin(phase / 6.0),
                    "orp": 250.0 + 40.0 * math.sin(phase / 5.0),
                    "updated": time.time(),
                })
        with self._lock:
            return {"authorized": self.authorized, "pump_channel": PUMP_CHANNEL,
                    "sensors": dict(self._sensor), "motor": dict(self._motor)}

    def set_pump(self, on: bool) -> Dict[str, Any]:
        if self.simulation:
            with self._lock:
                mask = int(self._sensor["relay_state"])
                mask = (mask | PUMP_BIT) if on else (mask & ~PUMP_BIT)
                self._sensor.update({"relay_state": mask, "pump_on": on,
                                     "status": "SIMULATION", "updated": time.time()})
            return {"ok": True, "simulation": True, "pump_on": on}
        if not self.authorized:
            raise RuntimeError("当前启动未授予采样硬件控制权限")
        with self._lock:
            if self._bus is None:
                raise RuntimeError("RS-485未连接：{}".format(self._sensor["status"]))
            if not self._sensor.get("relay_online"):
                raise RuntimeError("继电器状态未知，拒绝覆盖其余通道：{}".format(
                    self._sensor["status"]))
            current = int(self._sensor.get("relay_state") or 0)
            mask = (current | PUMP_BIT) if on else (current & ~PUMP_BIT)
            bits = [bool(mask & (1 << index)) for index in range(8)]
            self._bus.write_coils(4, 0, bits, "水泵{}".format("开" if on else "关"))
            self._sensor.update({"relay_state": mask, "pump_on": on,
                                 "status": self._bus.stats, "updated": time.time()})
        LOG.warning("水泵命令：继电器第4路 %s", "ON" if on else "OFF")
        return {"ok": True, "pump_on": on, "relay_state": mask}

    def motor_command(self, name: str, speed: float = 0.5,
                      duration: float = 3.0,
                      on_started: Optional[Callable[[], None]] = None
                      ) -> Dict[str, Any]:
        allowed = {"forward", "reverse", "dig", "dig_continuous", "stop", "status"}
        if name not in allowed:
            raise ValueError("不允许的电机命令：{}".format(name))
        if not 0.0 < float(speed) <= MOTOR_MAX_SPEED_RAD_S:
            raise ValueError("电机速度必须在0到1.0 rad/s之间")
        if not 0.0 < float(duration) <= MOTOR_MAX_DURATION_S:
            raise ValueError("电机运行时间必须在0到60秒之间")
        if on_started is not None and name != "dig_continuous":
            raise ValueError("启动回调仅适用于连续钻进命令")
        if self.simulation:
            if name == "dig_continuous":
                self._simulation_motor_stop.clear()
                with self._lock:
                    self._motor.update({"online": True, "busy": True,
                                        "command": name, "status": "SIMULATION连续旋转",
                                        "updated": time.time()})
                if on_started is not None:
                    on_started()
                self._simulation_motor_stop.wait()
                with self._lock:
                    self._motor.update({"busy": False, "status": "SIMULATION已停止并失能",
                                        "updated": time.time(), "velocity": 0.0})
                return {"ok": True, "simulation": True, "command": name}
            if name == "stop":
                self._simulation_motor_stop.set()
            with self._lock:
                self._motor.update({"online": True, "busy": False, "command": name,
                                    "status": "SIMULATION命令完成", "updated": time.time()})
            return {"ok": True, "simulation": True, "command": name}
        if not self.authorized:
            raise RuntimeError("当前启动未授予采样硬件控制权限")
        if not Path(self.motor_script).is_file():
            raise RuntimeError("电机控制程序不存在：{}".format(self.motor_script))
        if name == "stop":
            with self._lock:
                running = self._motor_process
            if running is not None and running.poll() is None:
                LOG.warning("停止命令正在中断电机动作进程")
                running.terminate()
                try:
                    running.wait(timeout=3.0)
                except subprocess.TimeoutExpired:
                    running.kill()
                    running.wait(timeout=1.0)
        acquired = (self._motor_lock.acquire(timeout=4.0) if name == "stop"
                    else self._motor_lock.acquire(False))
        if not acquired:
            raise RuntimeError("电机已有命令正在执行")
        try:
            script_name = "dig-continuous" if name == "dig_continuous" else name
            argv = [sys.executable, self.motor_script, script_name, "--port", self.motor_port]
            if name in {"forward", "reverse", "dig", "dig_continuous"}:
                argv += ["--speed", str(float(speed)), "--duration", str(float(duration)),
                         "--confirm-run", "YES"]
            if name == "dig_continuous":
                argv += ["--watchdog", str(self.motor_watchdog_timeout_s)]
            with self._lock:
                self._motor.update({"busy": True, "command": name,
                                    "status": "命令执行中", "updated": time.time()})
                if name in {"forward", "reverse", "dig", "dig_continuous"}:
                    self._motor["limits_verified"] = False
            process = subprocess.Popen(
                argv, stdin=subprocess.PIPE if name == "dig_continuous" else None,
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                universal_newlines=True, bufsize=1)
            with self._lock:
                self._motor_process = process
            if on_started is not None:
                on_started()
            try:
                if name == "dig_continuous":
                    process.wait()
                    stdout = process.stdout.read() if process.stdout is not None else ""
                else:
                    # Keep a small allowance for configuration and safe stop;
                    # the old fixed 15 s timeout aborted longer valid runs.
                    stdout, _ = process.communicate(timeout=float(duration) + 5.0)
            except subprocess.TimeoutExpired:
                process.terminate()
                try:
                    stdout, _ = process.communicate(timeout=3.0)
                except subprocess.TimeoutExpired:
                    process.kill()
                    stdout, _ = process.communicate(timeout=1.0)
                raise RuntimeError("电机命令超时；已中断并发送安全停止")
            output = stdout.strip()[-1200:]
            with self._lock:
                self._motor.update({"online": process.returncode == 0,
                                    "busy": False, "status": output or "无输出",
                                    "updated": time.time()})
                if name in {"forward", "reverse", "dig", "dig_continuous"}:
                    self._motor["limits_verified"] = (
                        process.returncode == 0 and
                        "safety: mode=VEL" in output and
                        "current_limit=4" in output and
                        "torque_limit=15" in output
                    )
                match = re.search(r"position=([-+0-9.]+).*velocity=([-+0-9.]+).*torque=([-+0-9.]+)", output)
                if match:
                    self._motor.update({"position": float(match.group(1)),
                                        "velocity": float(match.group(2)),
                                        "torque": float(match.group(3))})
            if process.returncode != 0:
                raise RuntimeError("电机命令失败：{}".format(output))
            LOG.warning("电机白名单命令完成：%s", name)
            return {"ok": True, "command": name, "output": output}
        finally:
            with self._lock:
                self._motor["busy"] = False
                self._motor_process = None
            self._motor_lock.release()

    def motor_keepalive(self) -> None:
        """Renew the low-level continuous-command watchdog."""
        if self.simulation:
            return
        with self._lock:
            process = self._motor_process
            stream = process.stdin if process is not None else None
        if process is None or process.poll() is not None or stream is None:
            raise RuntimeError("连续电机进程未运行")
        try:
            stream.write("KEEPALIVE\n")
            stream.flush()
        except (BrokenPipeError, OSError, ValueError) as error:
            raise RuntimeError("连续电机看门狗心跳发送失败：{}".format(error))

    def emergency_stop(self) -> None:
        errors = []
        try:
            self.set_pump(False)
        except Exception as error:
            errors.append("水泵：{}".format(error))
        try:
            self.motor_command("stop")
        except Exception as error:
            errors.append("电机：{}".format(error))
        if errors:
            LOG.error("采样硬件急停部分失败：%s", "; ".join(errors))

    def close(self) -> None:
        if self.authorized and not self.simulation:
            self.emergency_stop()
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        if self._bus is not None:
            self._bus.close()
