"""Tool5 drill workflow coordinator.

The arm motion remains inside rm75_vertical_press, the existing sole JointPos
source.  This class only coordinates that controller with one watchdog-protected
continuous motor process and exposes a small, testable Web state machine.
"""
from __future__ import annotations

import math
import threading
import time
from typing import Any, Callable, Dict, Optional


ACTIVE_STATES = {"STARTING", "ROTATING", "DRILLING", "DEPTH_REACHED",
                 "STOPPING_MOTOR"}


class Tool5DrillWorkflow:
    def __init__(
        self,
        config: Dict[str, Any],
        hardware: Any,
        telemetry: Callable[[], Dict[str, Any]],
        prepare_arm: Callable[[], Any],
        engage_arm: Callable[[float], Any],
        hold_arm: Callable[[], Any],
        safe_stop_arm: Callable[[], Any],
        lease_active: Callable[[], bool],
        simulation: bool = False,
    ) -> None:
        cfg = dict(config.get("tool5_drill", {}))
        self.max_feed_travel_m = float(cfg.get("max_feed_travel_m", 0.05))
        self.max_axial_force_n = float(cfg.get("max_axial_force_n", 60.0))
        self.max_torque_nm = float(cfg.get("max_torque_nm", 15.0))
        self.motor_speed_rad_s = float(cfg.get("motor_speed_rad_s", 0.5))
        self.motor_only = bool(cfg.get("motor_only", False))
        self.require_force_feedback = bool(cfg.get("require_force_feedback", False))
        self.motor_watchdog_timeout_s = float(cfg.get("motor_watchdog_timeout_s", 1.0))
        self.sim_feed_velocity_m_s = float(cfg.get("max_z_velocity_m_s", 0.002))
        self.startup_feedback_timeout_s = max(
            0.01, min(30.0, float(config.get("feedback_timeout_s", 5.0))))
        if not 0.25 <= self.motor_watchdog_timeout_s <= 2.0:
            raise ValueError("Tool5 motor watchdog must be between 0.25 and 2 seconds")
        self.hardware = hardware
        self.telemetry = telemetry
        self.prepare_arm = prepare_arm
        self.engage_arm = engage_arm
        self.hold_arm = hold_arm
        self.safe_stop_arm = safe_stop_arm
        self.lease_active = lease_active
        self.simulation = bool(simulation)
        self._lock = threading.RLock()
        self._output_lock = threading.Lock()
        self._startup_cancelled = threading.Event()
        self._motor_launch_event = threading.Event()
        self._motor_launch_error = ""
        self._motor_thread: Optional[threading.Thread] = None
        self._motor_running = False
        self._operator_stop = False
        self._started_monotonic = 0.0
        self._start_z: Optional[float] = None
        self._status: Dict[str, Any] = {
            "state": "IDLE", "target_depth_m": 0.0,
            "actual_depth_m": 0.0, "axial_force_n": None,
            "torque_norm_nm": None, "motor_running": False,
            "feed_state": "IDLE", "fault_reason": "",
            "tcp_reference": "Link7 relative Z; Tool5 tip offset pending calibration",
            "tcp_calibrated": False,
            "force_feedback_required": self.require_force_feedback,
            "feed_control": ("INDEPENDENT_FORCE_CONTROL" if self.motor_only
                             else "BOUNDED_POSITION"),
            "motor_watchdog_timeout_s": self.motor_watchdog_timeout_s,
        }

    def snapshot(self) -> Dict[str, Any]:
        self.poll()
        with self._lock:
            return dict(self._status)

    def start(self, depth_m: float = 0.0) -> Dict[str, Any]:
        depth = float(depth_m)
        if (not self.motor_only and
                (not math.isfinite(depth) or
                 not 0.0 < depth <= self.max_feed_travel_m)):
            raise ValueError("Tool5下降距离必须大于0且不超过独立最大进给行程")
        if self.motor_only:
            depth = 0.0
        with self._lock:
            if self._status["state"] == "COMPLETED":
                self._status.update({
                    "state": "IDLE", "target_depth_m": 0.0,
                    "actual_depth_m": 0.0, "feed_state": "IDLE",
                    "fault_reason": "", "motor_running": False,
                })
            if self._status["state"] != "IDLE":
                raise RuntimeError("Tool5任务不是IDLE；保持中的任务不能再次启动")
            self._status.update({
                "state": "STARTING", "target_depth_m": depth,
                "actual_depth_m": 0.0, "fault_reason": "",
                "feed_state": ("INDEPENDENT" if self.motor_only else "STARTING"),
                "motor_running": False,
            })
            self._operator_stop = False
            self._motor_launch_error = ""
            self._motor_launch_event.clear()
            self._startup_cancelled.clear()
        try:
            if self.motor_only:
                self._preflight_motor()
                self._ensure_starting()
                self._start_z = None
            else:
                raw = self._wait_for_preflight_feedback()
                self._preflight(raw)
                self._ensure_starting()
                self.prepare_arm()
                self._ensure_starting()
                raw = self.telemetry()
                self._ensure_starting()
                tcp = raw.get("tcp", {})
                self._start_z = (float(tcp.get("z", 0.0))
                                 if not self.simulation else 0.0)
            self._started_monotonic = time.monotonic()
            if not self.motor_only:
                with self._output_lock:
                    self._assert_starting_authorized()
                    self.engage_arm(depth)
                self._ensure_starting()
            worker = threading.Thread(target=self._run_motor,
                                      name="rm75-tool5-motor", daemon=True)
            with self._lock:
                self._motor_thread = worker
            worker.start()
            self._wait_for_motor_launch()
            with self._lock:
                if self._status["state"] != "STARTING":
                    raise RuntimeError("Tool5启动已中止：{}".format(
                        self._status.get("fault_reason") or self._status["state"]))
                self._status.update({
                    "state": "ROTATING" if self.motor_only else "DRILLING",
                    "feed_state": "INDEPENDENT" if self.motor_only else "FEEDING",
                })
            return self.snapshot()
        except Exception as error:
            self._safe_stop("Tool5启动失败：{}".format(error))
            raise

    def stop_rotation(self) -> Dict[str, Any]:
        with self._lock:
            if self._status["state"] not in ACTIVE_STATES:
                raise RuntimeError("Tool5电机当前没有活动任务")
            self._operator_stop = True
            self._status["state"] = "STOPPING_MOTOR"
            motor_thread = self._motor_thread
        errors = []
        if not self.motor_only:
            try:
                self.hold_arm()
            except Exception as error:
                errors.append("机械臂保持失败：{}".format(error))
        try:
            self.hardware.motor_command("stop")
            self.hardware.motor_command("status")
            motor = self.hardware.snapshot().get("motor", {})
            velocity = motor.get("velocity")
            if (not self.simulation and
                    (not motor.get("online") or velocity is None or
                     abs(float(velocity)) > 0.05)):
                raise RuntimeError("停止后状态回读未确认零速")
            if (motor_thread is not None and motor_thread is not threading.current_thread()):
                motor_thread.join(timeout=1.0)
                if motor_thread.is_alive():
                    raise RuntimeError("电机停止后工作线程未退出")
        except Exception as error:
            errors.append("电机停止失败：{}".format(error))
        if errors:
            reason = "; ".join(errors)
            self._safe_stop(reason)
            raise RuntimeError(reason)
        with self._lock:
            self._motor_running = False
            self._status.update({
                "state": "COMPLETED", "motor_running": False,
                "feed_state": ("INDEPENDENT" if self.motor_only else "HOLDING"),
                "fault_reason": "",
            })
        return self.snapshot()

    def reset(self) -> None:
        with self._lock:
            if self._status["state"] in ACTIVE_STATES:
                raise RuntimeError("活动Tool5任务不能复位")
            self._status.update({
                "state": "IDLE", "target_depth_m": 0.0,
                "actual_depth_m": 0.0, "feed_state": "IDLE",
                "fault_reason": "", "motor_running": False,
            })

    def emergency_stop(self, reason: str = "软件急停", stop_arm: bool = True) -> None:
        self._safe_stop(reason, stop_arm=stop_arm)

    def poll(self) -> None:
        with self._lock:
            state = self._status["state"]
        if state not in ACTIVE_STATES:
            return
        if not self.lease_active():
            self._safe_stop("浏览器控制租约或心跳已失效")
            return
        with self._lock:
            motor_running = self._motor_running
            motor_age = time.monotonic() - self._started_monotonic
        if state == "STARTING" and not motor_running:
            return
        if motor_running:
            try:
                self.hardware.motor_keepalive()
            except Exception as error:
                if motor_age >= 0.5:
                    self._safe_stop(str(error))
                    return
        if self.motor_only:
            return
        raw = self.telemetry()
        try:
            self._check_runtime_safety(raw)
        except Exception as error:
            self._safe_stop(str(error))
            return
        if self.simulation:
            elapsed = max(0.0, time.monotonic() - self._started_monotonic)
            depth = min(float(self._status["target_depth_m"]),
                        elapsed * self.sim_feed_velocity_m_s)
            node_state = "DEPTH_REACHED" if depth >= self._status["target_depth_m"] else "DRILLING"
        else:
            tcp = raw.get("tcp", {})
            actual_z = float(tcp["z"])
            depth = max(0.0, float(self._start_z) - actual_z)
            node_state = str(raw.get("tool5_drill_state") or "")
        with self._lock:
            self._status["actual_depth_m"] = depth
            if node_state == "SAFE_STOP":
                reason = str(raw.get("stop_reason") or "底层力控SAFE_STOP")
            else:
                reason = ""
            if (depth >= self._status["target_depth_m"] or
                    node_state == "DEPTH_REACHED") and self._status["state"] == "DRILLING":
                self._status.update({"state": "DEPTH_REACHED", "feed_state": "HOLDING"})
        if reason:
            self._safe_stop(reason)

    def _wait_for_preflight_feedback(self) -> Dict[str, Any]:
        deadline = time.monotonic() + self.startup_feedback_timeout_s
        while True:
            self._ensure_starting()
            raw = self.telemetry()
            self._ensure_starting()
            unavailable = []
            if not self.simulation and not raw.get("health", {}).get("driver"):
                unavailable.append("driver")
            fresh = raw.get("fresh", {})
            required_feedback = ["joints", "tcp"]
            if self.require_force_feedback:
                required_feedback.append("wrench")
            unavailable.extend(key for key in required_feedback if not fresh.get(key))
            if not unavailable:
                return raw
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                raise RuntimeError(
                    "Tool5启动反馈等待超时：{}".format(", ".join(unavailable)))
            time.sleep(min(0.05, remaining))

    def _wait_for_motor_launch(self) -> None:
        deadline = time.monotonic() + self.startup_feedback_timeout_s
        while not self._motor_launch_event.wait(timeout=0.01):
            self._ensure_starting()
            if time.monotonic() >= deadline:
                raise RuntimeError("Tool5电机启动确认超时")
        self._ensure_starting()
        with self._lock:
            error = self._motor_launch_error
            motor_running = self._motor_running
        if error:
            raise RuntimeError(error)
        if not motor_running:
            raise RuntimeError("Tool5电机未确认启动")

    def _ensure_starting(self) -> None:
        with self._lock:
            state = self._status["state"]
            reason = str(self._status.get("fault_reason") or state)
        if self._startup_cancelled.is_set() or state != "STARTING":
            raise RuntimeError("Tool5启动已中止：{}".format(reason))
        if not self.lease_active():
            reason = "浏览器控制租约或心跳已失效"
            self._safe_stop(reason)
            raise RuntimeError("Tool5启动已中止：{}".format(reason))

    def _assert_starting_authorized(self) -> None:
        with self._lock:
            state = self._status["state"]
            reason = str(self._status.get("fault_reason") or state)
        if (self._startup_cancelled.is_set() or state != "STARTING" or
                not self.lease_active()):
            raise RuntimeError("Tool5启动已中止：{}".format(reason))

    def _preflight(self, raw: Dict[str, Any]) -> None:
        motor = self.hardware.snapshot().get("motor", {})
        if not motor.get("online"):
            self.hardware.motor_command("status")
            motor = self.hardware.snapshot().get("motor", {})
        if not motor.get("online"):
            raise RuntimeError("DM-J4310通信不可用")
        health = raw.get("health", {})
        fresh = raw.get("fresh", {})
        if not self.simulation and not health.get("driver"):
            raise RuntimeError("RM75驱动未就绪")
        required_feedback = ["joints", "tcp"]
        if self.require_force_feedback:
            required_feedback.append("wrench")
        for key in required_feedback:
            if not fresh.get(key):
                raise RuntimeError("{}反馈不可用或过期".format(key))
        self._check_runtime_safety(raw)

    def _preflight_motor(self) -> None:
        motor = self.hardware.snapshot().get("motor", {})
        if not motor.get("online"):
            self.hardware.motor_command("status")
            motor = self.hardware.snapshot().get("motor", {})
        if not motor.get("online"):
            raise RuntimeError("DM-J4310通信不可用")

    def _check_runtime_safety(self, raw: Dict[str, Any]) -> None:
        if raw.get("driver_arm_error") or raw.get("driver_system_error"):
            raise RuntimeError("RM75驱动报告ArmError或SysError")
        if raw.get("driver_plan_ok") is False:
            raise RuntimeError("RM75驱动跟踪/规划状态异常")
        fresh = raw.get("fresh", {})
        for key in ("joints", "tcp"):
            if not fresh.get(key):
                raise RuntimeError("{}反馈超时".format(key))
        if self.require_force_feedback and not fresh.get("wrench"):
            raise RuntimeError("wrench反馈超时")
        reason = str(raw.get("stop_reason") or "")
        if reason and any(word in reason.lower() for word in
                          ("tracking", "跟踪", "timeout", "超时", "failed", "故障")):
            raise RuntimeError(reason)
        if not fresh.get("wrench"):
            with self._lock:
                self._status["axial_force_n"] = None
                self._status["torque_norm_nm"] = None
                self._status["feed_control"] = "BOUNDED_POSITION"
            return
        wrench = raw.get("wrench", {})
        fz = float(wrench.get("Fz"))
        torques = [float(wrench.get(key)) for key in ("Tx", "Ty", "Tz")]
        torque_norm = math.sqrt(sum(value * value for value in torques))
        if not math.isfinite(fz) or not math.isfinite(torque_norm):
            raise RuntimeError("六维力数据非有限值")
        with self._lock:
            filtered = raw.get("filtered_force_z")
            self._status["axial_force_n"] = (
                float(filtered) if filtered is not None and
                math.isfinite(float(filtered)) else fz)
            self._status["torque_norm_nm"] = torque_norm
            self._status["feed_control"] = "BOUNDED_POSITION_WITH_FORCE_GUARD"
        if abs(fz) > self.max_axial_force_n:
            raise RuntimeError("轴向力超过Tool5限制")
        if torque_norm > self.max_torque_nm:
            raise RuntimeError("力矩超过Tool5限制")

    def _run_motor(self) -> None:
        output_lock_held = False

        def motor_started() -> None:
            nonlocal output_lock_held
            with self._lock:
                self._motor_running = True
                self._status["motor_running"] = True
            self._output_lock.release()
            output_lock_held = False
            self._motor_launch_event.set()

        try:
            self._output_lock.acquire()
            output_lock_held = True
            self._assert_starting_authorized()
            self.hardware.motor_command(
                "dig_continuous", speed=self.motor_speed_rad_s,
                on_started=motor_started)
            with self._lock:
                stopped_by_user = self._operator_stop
                self._motor_running = False
                self._status["motor_running"] = False
            if not stopped_by_user:
                self._safe_stop("DM-J4310连续命令意外结束")
        except Exception as error:
            if output_lock_held:
                self._output_lock.release()
                output_lock_held = False
            with self._lock:
                stopped_by_user = self._operator_stop
                if not self._motor_launch_event.is_set():
                    self._motor_launch_error = str(error)
                self._motor_running = False
                self._status["motor_running"] = False
            self._motor_launch_event.set()
            if not stopped_by_user:
                self._safe_stop("DM-J4310命令失败：{}".format(error))

    def _safe_stop(self, reason: str, stop_arm: bool = True) -> None:
        self._startup_cancelled.set()
        with self._output_lock:
            with self._lock:
                if self._status["state"] == "SAFE_STOP":
                    return
                self._status.update({
                    "state": "SAFE_STOP", "feed_state": "SAFE_STOP",
                    "fault_reason": str(reason), "motor_running": False,
                })
                self._operator_stop = True
            try:
                self.hardware.motor_command("stop")
            except Exception as error:
                with self._lock:
                    self._status["fault_reason"] += "; 电机停止失败：{}".format(error)
        if stop_arm:
            try:
                self.safe_stop_arm()
            except Exception:
                pass
