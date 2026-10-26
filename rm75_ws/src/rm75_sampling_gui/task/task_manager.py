"""High-level orchestration over existing RM75 ROS interfaces.

The task manager never generates joint trajectories. It starts the existing
Touch node, calls the existing QR task service, and delegates voltage/stop
commands to :class:`ROSInterface`, where fresh acknowledgements are required.
"""

import logging
import os
import signal
import subprocess
import threading
from typing import Any, Dict, List, Optional

import rosnode
import rospkg

from rm75_sampling_gui.ros.ros_interface import ROSInterface
from rm75_sampling_gui.state.state_machine import RobotState, StateMachine
from rm75_sampling_gui.task.system_manager import SystemLaunchManager


LOGGER = logging.getLogger(__name__)


class TaskManagerError(RuntimeError):
    """Raised when an HMI workflow precondition or delegated task fails."""


class ControlLaunchManager:
    """Own mutually exclusive Touch/force-control roslaunch children."""

    def __init__(self, config: Dict[str, Any], execution_enabled: bool) -> None:
        self._config = config
        self._execution_enabled = bool(execution_enabled)
        self._parent: Optional[subprocess.Popen] = None
        self._mode: Optional[str] = None
        self._lock = threading.RLock()
        self._rospack = rospkg.RosPack()

    @property
    def mode(self) -> Optional[str]:
        with self._lock:
            if self._parent is not None and self._parent.poll() is not None:
                self._parent = None
                self._mode = None
            return self._mode

    def set_execution_enabled(self, enabled: bool) -> None:
        with self._lock:
            if self._parent is not None and self._parent.poll() is None:
                raise TaskManagerError("控制节点运行时禁止切换执行授权")
            self._execution_enabled = bool(enabled)

    def start(self, mode: str) -> None:
        """Start teleop or the reviewed vertical force-control node."""

        if mode not in ("teleop", "z_force_teleop", "force_control", "tool5_drill"):
            raise TaskManagerError("unsupported control launch mode {}".format(mode))
        with self._lock:
            if self._parent is not None:
                if self._parent.poll() is None:
                    if self._mode == mode:
                        return
                    raise TaskManagerError(
                        "control launch {} is already active".format(self._mode)
                    )
                self._parent = None
                self._mode = None

            live = set(rosnode.get_node_names())
            conflicts = sorted(
                node
                for node in (
                    "/rm75_touch_teleop",
                    "/rm75_vertical_press",
                    "/rm_control",
                    "/joint_control_server",
                    "/trajectory_bridge",
                )
                if node in live
            )
            if conflicts:
                raise TaskManagerError(
                    "competing control node is live: {}".format(", ".join(conflicts))
                )
            required = {"/rm_driver", "/robot_state_publisher"}
            missing = sorted(required - live)
            if missing:
                raise TaskManagerError(
                    "base RM75 stack is incomplete: missing {}".format(", ".join(missing))
                )

            touch_driver_live = "/Geomagic/Geomagic" in live
            helper_nodes = {
                "/touch_pubforce",
                "/rm75_get_end_effector_pose",
            }
            live_helpers = helper_nodes.intersection(live)
            if live_helpers and live_helpers != helper_nodes:
                raise TaskManagerError(
                    "Touch helper节点只启动了一部分：{}；请先恢复一致状态".format(
                        ", ".join(sorted(live_helpers))
                    )
                )
            start_helpers = not live_helpers

            if mode in ("teleop", "z_force_teleop"):
                package_name = "rm75_touch_teleop"
                launch_name = "teleop.launch"
                config_key = (
                    "z_force_teleop_config"
                    if mode == "z_force_teleop"
                    else "teleop_config"
                )
            else:
                if not self._execution_enabled:
                    raise TaskManagerError(
                        "真实执行未授权，拒绝以enable_motion:=true启动垂直下压力控"
                    )
                package_name = "rm75_vertical_press"
                launch_name = "vertical_press_real.launch"
                config_key = "force_control_config"
            package_path = self._rospack.get_path(package_name)
            launch_path = os.path.join(package_path, "launch", launch_name)
            config_path = str(self._config[config_key])
            if not os.path.isabs(config_path):
                config_path = os.path.join(package_path, config_path)
            if not os.path.isfile(config_path):
                raise TaskManagerError("control config not found: {}".format(config_path))

            arguments = ["config:={}".format(config_path)]
            if mode in ("teleop", "z_force_teleop"):
                arguments.extend([
                    "motion_enabled:={}".format(
                        "true" if self._execution_enabled else "false"
                    ),
                    "haptic_feedback:={}".format(
                        "true" if mode == "z_force_teleop" else "false"
                    ),
                ])
            else:
                arguments.extend([
                    "enable_motion:={}".format(
                        "true"
                    ),
                    # The HMI base stack already owns these two nodes.
                    "start_driver:=false",
                    "start_robot_state_publisher:=false",
                ])
                if mode == "tool5_drill":
                    drill = dict(self._config.get("tool5_drill", {}))
                    arguments.extend([
                        "tool5_drill_enabled:=true",
                        "tool5_drill_max_feed_travel:={}".format(
                            float(drill.get("max_feed_travel_m", 0.05))),
                        "desired_force:={}".format(
                            float(drill.get("desired_force_n", -10.0))),
                        "max_z_velocity:={}".format(
                            float(drill.get("max_z_velocity_m_s", 0.002))),
                        "force_limit:={}".format(
                            float(drill.get("max_axial_force_n", 60.0))),
                        "max_external_force:={}".format(
                            float(drill.get("max_axial_force_n", 60.0))),
                        "max_external_torque:={}".format(
                            float(drill.get("max_torque_nm", 5.0))),
                    ])
            arguments.extend([
                "start_touch_driver:={}".format(
                    "false" if mode == "tool5_drill" or touch_driver_live else "true"
                ),
                "start_touch_helpers:={}".format(
                    "true" if mode != "tool5_drill" and start_helpers else "false"
                ),
            ])
            # ROSLaunchParent installs signal handlers and cannot run in the
            # Qt worker thread. Give the existing roslaunch its own process.
            parent = subprocess.Popen(
                ["roslaunch", launch_path] + arguments, start_new_session=True)
            self._parent = parent
            self._mode = mode
            LOGGER.info(
                "已启动%s，mode=%s，motion_enabled=%s",
                package_name,
                mode,
                self._execution_enabled,
            )

    def stop(self) -> None:
        """Stop only the control launch child owned by this GUI."""

        with self._lock:
            parent = self._parent
            mode = self._mode
        if parent is not None:
            if parent.poll() is None:
                os.killpg(parent.pid, signal.SIGINT)
                try:
                    parent.wait(timeout=5.0)
                except subprocess.TimeoutExpired:
                    os.killpg(parent.pid, signal.SIGTERM)
                    parent.wait(timeout=3.0)
            with self._lock:
                if self._parent is parent:
                    self._parent = None
                    self._mode = None
            LOGGER.info("已停止GUI管理的控制节点，mode=%s", mode)


class TaskManager:
    """Serialize operator workflows while retaining existing safety gates."""

    def __init__(
        self,
        state_machine: StateMachine,
        ros_interface: ROSInterface,
        tools: List[Dict[str, Any]],
        config: Dict[str, Any],
        execution_enabled: bool = False,
        launch_manager: Optional[ControlLaunchManager] = None,
        system_manager: Optional[SystemLaunchManager] = None,
        joint_adapter: Any = None,
    ) -> None:
        self.state_machine = state_machine
        self.ros = ros_interface
        self.execution_enabled = bool(execution_enabled)
        self.config = config
        self.tools = {str(tool["name"]): dict(tool) for tool in tools}
        self.selected_tool: Optional[str] = None
        self._operation_lock = threading.RLock()
        self.launches = launch_manager or ControlLaunchManager(
            config, self.execution_enabled
        )
        self.systems = system_manager or SystemLaunchManager(
            config, self.execution_enabled
        )
        self.joint_adapter = joint_adapter

    def set_control_path(self, path: str) -> Dict[str, Any]:
        """Atomically select one authorized control path while all systems stop."""

        with self._operation_lock:
            if self.state_machine.state != RobotState.IDLE:
                raise TaskManagerError(
                    "仅IDLE状态允许切换控制路径；当前为{}".format(
                        self.state_machine.state.value
                    )
                )
            if self.launches.mode is not None:
                raise TaskManagerError("切换控制路径前必须停止Touch或力控节点")
            self.systems.set_control_path(path)
            enabled = path != "observe"
            values = {
                "execution_enabled": enabled,
                "joint_control_execution_authorized": path == "joint_control",
                "touch_z_force_authorized": path == "z_force",
                "tool5_drill_authorized": path == "tool5_drill",
                "trajectory_execution_authorized": path in {"pose_pickup", "qr_pickup"},
                "qr_execution_authorized": path == "qr_pickup",
            }
            self.config.update(values)
            self.execution_enabled = enabled
            self.launches.set_execution_enabled(enabled)
            if self.joint_adapter is not None:
                self.joint_adapter.execution_authorized = path == "joint_control"
                self.joint_adapter.trajectory_execution_authorized = path in {
                    "pose_pickup", "qr_pickup"
                }
                self.joint_adapter.supervised_commissioning_authorized = bool(
                    self.config.get("supervised_commissioning_authorized", False)
                )
            LOGGER.warning("Web统一控制路径切换为%s；当前没有启动运动命令源", path)
            return {"control_path": path, "execution": dict(values)}

    def start_subsystem(self, name: str) -> Dict[str, Any]:
        """Start an existing ROS subsystem without changing robot task state."""

        with self._operation_lock:
            return self.systems.start(name)

    def stop_subsystem(self, name: str) -> Dict[str, Any]:
        """Stop only a subsystem launch owned by this HMI."""

        with self._operation_lock:
            if name == "base":
                if self.launches.mode is not None:
                    raise TaskManagerError("请先停止Touch遥操作或力控节点")
                statuses = self.systems.status()
                active = [item for item in ("qr", "camera")
                          if statuses.get(item, {}).get("owned")]
                if active:
                    raise TaskManagerError(
                        "请先停止：{}".format(", ".join(active))
                    )
            return self.systems.stop(name)

    def subsystem_status(self) -> Dict[str, Dict[str, Any]]:
        return self.systems.status()

    @property
    def control_mode(self) -> Optional[str]:
        """Return the actual GUI-owned control launch mode, if any."""

        if self.joint_adapter is not None and self.joint_adapter.active:
            return "joint_control"
        mode = self.launches.mode
        return mode if mode in ("teleop", "z_force_teleop", "force_control", "tool5_drill") else None

    def validate_joint_target(self, target: Any) -> Dict[str, Any]:
        """Run the adapter checks in the caller's worker thread."""

        if self.joint_adapter is None:
            raise TaskManagerError("关节控制适配器未加载")
        with self._operation_lock:
            self._ensure_not_safe_stop()
            return self.joint_adapter.validate(
                target,
                self.ros.telemetry_snapshot(),
                self.state_machine.state,
                self.control_mode,
                for_execution=True,
            )

    def execute_joint_target(self, target: Any) -> Dict[str, Any]:
        """Validate and execute one click through the guarded trajectory adapter."""

        if self.joint_adapter is None:
            raise TaskManagerError("关节控制适配器未加载")
        with self._operation_lock:
            self._ensure_not_safe_stop()
            if self.launches.mode is not None:
                raise TaskManagerError("Touch或力控正在占用机械臂")
            validation = self.joint_adapter.validate(
                target,
                self.ros.telemetry_snapshot(),
                self.state_machine.state,
                self.control_mode,
                for_execution=True,
            )
            if not validation.get("ok") or not validation.get("token"):
                failures = [
                    str(item.get("detail", item.get("name", "检查失败")))
                    for item in validation.get("checks", []) if not item.get("ok")
                ]
                raise TaskManagerError(
                    "七轴目标安全检查未通过：{}".format(
                        "；".join(failures) or "未知原因"
                    )
                )
            token = str(validation["token"])
            try:
                self.state_machine.transition(
                    RobotState.JOINT_CONTROL, "开始受控七轴目标执行"
                )
                result = self.joint_adapter.execute(
                    token, target, self.ros.telemetry_snapshot
                )
                if result.get("cancelled"):
                    self.state_machine.fail(
                        RuntimeError("七轴执行已取消并请求软件急停"),
                        "七轴目标取消",
                    )
                    return result
                self.state_machine.transition(RobotState.IDLE, "七轴目标执行结束")
                return result
            except Exception as error:
                self._fail_safe(error, "七轴目标执行")
                raise

    def cancel_joint_control(self) -> Dict[str, Any]:
        if self.joint_adapter is None:
            raise TaskManagerError("关节控制适配器未加载")
        return self.joint_adapter.cancel()

    def select_tool(self, tool_name: str) -> None:
        if tool_name not in self.tools:
            raise TaskManagerError("unknown tool {}".format(tool_name))
        self.selected_tool = tool_name
        LOGGER.info("任务管理器选择工具：%s", tool_name)

    def start_teleop(self) -> Dict[str, Any]:
        with self._operation_lock:
            self._ensure_not_safe_stop()
            if self.joint_adapter is not None and self.joint_adapter.active:
                raise TaskManagerError("七轴关节控制正在执行，拒绝Touch抢占")
            if "/joint_control_server" in set(rosnode.get_node_names()):
                raise TaskManagerError(
                    "基础栈当前处于七轴真机控制模式；请停止基础栈并以关节控制授权关闭的模式重启后再使用Touch"
                )
            if not self.systems.status().get("base", {}).get("running", False):
                raise TaskManagerError("请先在界面启动RM75基础栈")
            try:
                self.launches.start("teleop")
                self.ros.wait_for_node(
                    "/rm75_touch_teleop", self.config["node_start_timeout_s"]
                )
                self.state_machine.transition(RobotState.TELEOP, "Touch遥操作已就绪")
                return {"mode": "teleop", "execution_enabled": self.execution_enabled}
            except Exception as error:
                self._fail_safe(error, "启动Touch遥操作")
                raise

    def stop_teleop(self) -> Dict[str, Any]:
        with self._operation_lock:
            if self.launches.mode != "teleop":
                raise TaskManagerError("Touch遥操作节点当前未由界面启动")
            try:
                self.launches.stop()
                self.state_machine.transition(RobotState.IDLE, "Touch遥操作已停止")
                return {"stopped": True, "mode": "teleop"}
            except Exception as error:
                self._fail_safe(error, "停止Touch遥操作")
                raise

    def start_z_force_teleop(self) -> Dict[str, Any]:
        """Start the dedicated Z-only Touch hybrid force/haptic controller."""

        with self._operation_lock:
            self._ensure_not_safe_stop()
            if not self.execution_enabled or not self.config.get(
                    "touch_z_force_authorized", False):
                raise TaskManagerError(
                    "Z轴力位遥操作未授权；请使用z_force控制路径启动Web平台"
                )
            if self.joint_adapter is not None and self.joint_adapter.active:
                raise TaskManagerError("七轴关节控制正在执行，拒绝Touch抢占")
            live = set(rosnode.get_node_names())
            conflicting = sorted(node for node in (
                "/joint_control_server", "/trajectory_bridge", "/rm75_vertical_press"
            ) if node in live)
            if conflicting:
                raise TaskManagerError(
                    "存在冲突控制源，拒绝Z轴力位遥操作：{}".format(
                        ", ".join(conflicting))
                )
            if not self.systems.status().get("base", {}).get("running", False):
                raise TaskManagerError("请先在界面启动RM75基础栈")
            try:
                self.launches.start("z_force_teleop")
                self.ros.wait_for_node(
                    "/rm75_touch_teleop", self.config["node_start_timeout_s"]
                )
                self.ros.wait_for_force_feedback(self.config["feedback_timeout_s"])
                self.state_machine.transition(
                    RobotState.FORCE_CONTROL,
                    "Z轴力位遥操作与Touch力反馈已就绪",
                )
                return {
                    "mode": "z_force_teleop",
                    "execution_enabled": True,
                    "translation_axes": ["z"],
                    "orientation_locked": True,
                    "haptic_feedback": True,
                }
            except Exception as error:
                self._fail_safe(error, "启动Z轴力位遥操作")
                raise

    def stop_z_force_teleop(self) -> Dict[str, Any]:
        with self._operation_lock:
            if self.launches.mode != "z_force_teleop":
                raise TaskManagerError("Z轴力位遥操作当前未由界面启动")
            if self.ros.telemetry_snapshot().get("teleop_active", False):
                raise TaskManagerError(
                    "请先松开Touch灰键使active=false；需要立即停止时请使用急停"
                )
            try:
                self.launches.stop()
                self.state_machine.transition(
                    RobotState.IDLE, "Z轴力位遥操作与Touch力反馈已停止"
                )
                return {"stopped": True, "mode": "z_force_teleop"}
            except Exception as error:
                self._fail_safe(error, "停止Z轴力位遥操作")
                raise

    def detect_tool(self) -> Dict[str, Any]:
        with self._operation_lock:
            self._ensure_not_safe_stop()
            try:
                if self.launches.mode in ("teleop", "z_force_teleop"):
                    if self.ros.telemetry_snapshot().get("teleop_active", False):
                        raise TaskManagerError("请先释放Touch按键再识别工具")
                    self.launches.stop()
                self.state_machine.transition(RobotState.TOOL_DETECT, "开始Mark识别")
                result = self.ros.detect_tool(
                    list(self.tools.values()),
                    preferred=self.selected_tool,
                    timeout=float(self.config["marker_timeout_s"]),
                )
                self.selected_tool = str(result["name"])
                LOGGER.info("Mark识别完成：%s", self.selected_tool)
                return result
            except Exception as error:
                self._fail_safe(error, "Mark识别")
                raise

    def detect_common_marker(self) -> Dict[str, Any]:
        """Detect only the shared rack marker; do not change the selected tool."""

        with self._operation_lock:
            self._ensure_not_safe_stop()
            return self.ros.call_five_target_task(
                "detect", self._selected_tool()["slot_id"], execute=False)

    def calibrate_tool_rack(self) -> Dict[str, Any]:
        with self._operation_lock:
            self._ensure_not_safe_stop()
            return self.ros.call_five_target_task(
                "calibrate_reference", self._selected_tool()["slot_id"],
                execute=False)

    def confirm_five_target_validation(self) -> Dict[str, Any]:
        """Persist an operator's completed on-site validation; never moves."""

        with self._operation_lock:
            self._ensure_not_safe_stop()
            if not self.config.get("supervised_commissioning_authorized", False):
                raise TaskManagerError("现场验收确认未授权")
            return self.ros.call_five_target_task(
                "confirm_physical_validation",
                self._selected_tool()["slot_id"], execute=False,
                supervised_confirm=True)

    def compute_five_targets(self) -> Dict[str, Any]:
        with self._operation_lock:
            self._ensure_not_safe_stop()
            return self.ros.call_five_target_task(
                "compute_targets", self._selected_tool()["slot_id"],
                execute=False)

    def preview_selected_target(self) -> Dict[str, Any]:
        with self._operation_lock:
            self._ensure_not_safe_stop()
            return self.ros.call_five_target_task(
                "preview", self._selected_tool()["slot_id"], execute=False)

    def plan_selected_target(self) -> Dict[str, Any]:
        with self._operation_lock:
            self._ensure_not_safe_stop()
            if self.launches.mode is not None:
                raise TaskManagerError("Touch或力控正在占用机械臂")
            if self.joint_adapter is not None and self.joint_adapter.active:
                raise TaskManagerError("七轴受控执行正在占用机械臂")
            return self.ros.call_five_target_task(
                "plan", self._selected_tool()["slot_id"], execute=False)

    def execute_selected_target(self) -> Dict[str, Any]:
        with self._operation_lock:
            self._ensure_not_safe_stop()
            if self.launches.mode is not None:
                raise TaskManagerError("Touch或力控正在占用机械臂")
            if self.joint_adapter is not None and self.joint_adapter.active:
                raise TaskManagerError("七轴受控执行正在占用机械臂")
            if not self.execution_enabled or not self.config.get(
                    "qr_execution_authorized", False) or not self.config.get(
                        "trajectory_execution_authorized", False):
                raise TaskManagerError("公共标记自动取工具真机执行未授权")
            slot_id = int(self._selected_tool()["slot_id"])
            self.state_machine.transition(
                RobotState.TOOL_PICK, "J7观察位后自动取Tool{}".format(slot_id))
            try:
                result = self.ros.call_five_target_task(
                    "execute_selected", slot_id, execute=True)
                if int(result.get("holding_slot", -1)) != slot_id:
                    raise TaskManagerError("取工具结束但未记录slot {}".format(slot_id))
                self.state_machine.transition(
                    RobotState.TOOL_HOLD,
                    "Tool{} 24V ACK完成并退回观察位".format(slot_id))
                return result
            except Exception as error:
                self._fail_safe(error, "公共标记自动取工具")
                raise

    def plan_configured_tool(self, slot_id: int) -> Dict[str, Any]:
        """Dry-run the saved absolute Link7 pose through the existing MoveIt server."""

        with self._operation_lock:
            self._ensure_not_safe_stop()
            if slot_id not in range(1, 6):
                raise TaskManagerError("工具编号必须是1..5")
            if self.launches.mode is not None:
                raise TaskManagerError("Touch或力控正在占用机械臂")
            if self.joint_adapter is not None and self.joint_adapter.active:
                raise TaskManagerError("七轴受控执行正在占用机械臂")
            self.select_tool("Tool{}".format(slot_id))
            return self.ros.call_five_target_task(
                "plan_configured", slot_id, execute=False)

    def execute_configured_tool(self, slot_id: int) -> Dict[str, Any]:
        """Execute the previously dry-run configured pickup with all existing gates."""

        with self._operation_lock:
            self._ensure_not_safe_stop()
            if slot_id not in range(1, 6):
                raise TaskManagerError("工具编号必须是1..5")
            if self.launches.mode is not None:
                raise TaskManagerError("Touch或力控正在占用机械臂")
            if self.joint_adapter is not None and self.joint_adapter.active:
                raise TaskManagerError("七轴受控执行正在占用机械臂")
            if not self.execution_enabled or not self.config.get(
                    "trajectory_execution_authorized", False):
                raise TaskManagerError("五工具绝对位姿取物真机执行未授权")
            self.select_tool("Tool{}".format(slot_id))
            self.state_machine.transition(
                RobotState.TOOL_PICK, "执行Tool{}已保存取物位姿".format(slot_id))
            try:
                result = self.ros.call_five_target_task(
                    "execute_configured", slot_id, execute=True,
                    supervised_confirm=True)
                if not result.get("executed"):
                    raise TaskManagerError("配置取物任务未报告执行完成")
                self.state_machine.transition(
                    RobotState.TOOL_HOLD,
                    "Tool{}运动完成；物体在位未独立确认".format(slot_id))
                return result
            except Exception as error:
                self._fail_safe(error, "五工具配置取物")
                raise

    def move_to_camera_observation(self) -> Dict[str, Any]:
        """Move to the taught seven-joint camera pose and reacquire ID 0."""

        with self._operation_lock:
            self._ensure_not_safe_stop()
            if self.launches.mode is not None:
                raise TaskManagerError("Touch或力控正在占用机械臂")
            if self.joint_adapter is not None and self.joint_adapter.active:
                raise TaskManagerError("七轴受控执行正在占用机械臂")
            if not self.execution_enabled or not self.config.get(
                    "qr_execution_authorized", False) or not self.config.get(
                        "trajectory_execution_authorized", False):
                raise TaskManagerError("观察位真机执行未授权")
            slot_id = int(self._selected_tool()["slot_id"])
            self.state_machine.transition(
                RobotState.TOOL_DETECT, "移动到二维码观察位")
            try:
                result = self.ros.call_five_target_task(
                    "move_camera_observation", slot_id, execute=True)
                if int(result.get("voltage_v", -1)) != 0:
                    raise TaskManagerError("观察位完成但工具电压不是0V")
                return result
            except Exception as error:
                self._fail_safe(error, "移动到二维码观察位")
                raise

    def return_selected_target(self) -> Dict[str, Any]:
        """Return the recorded held tool through the five-target server."""

        with self._operation_lock:
            self._ensure_not_safe_stop()
            if self.launches.mode is not None:
                raise TaskManagerError("Touch或力控正在占用机械臂")
            if self.joint_adapter is not None and self.joint_adapter.active:
                raise TaskManagerError("七轴受控执行正在占用机械臂")
            if not self.execution_enabled or not self.config.get(
                    "qr_execution_authorized", False) or not self.config.get(
                        "trajectory_execution_authorized", False):
                raise TaskManagerError("公共标记归还工具真机执行未授权")
            slot_id = int(self._selected_tool()["slot_id"])
            self.state_machine.transition(
                RobotState.TOOL_RETURN,
                "20cm高位下降18cm归还Tool{}".format(slot_id))
            try:
                result = self.ros.call_five_target_task(
                    "return_held_tool", slot_id, execute=True,
                    supervised_confirm=True)
                if int(result.get("voltage_v", -1)) != 0:
                    raise TaskManagerError("归还结束但工具电压不是0V")
                if result.get("holding_slot") is not None:
                    raise TaskManagerError("归还结束但仍记录持有工具")
                self.state_machine.transition(
                    RobotState.IDLE,
                    "Tool{} 已释放并返回观察位".format(slot_id))
                return result
            except Exception as error:
                self._fail_safe(error, "公共标记归还工具")
                raise

    def confirm_selected_tool_held(self) -> Dict[str, Any]:
        """Recover held-tool state after a supervised HMI/ROS restart."""

        with self._operation_lock:
            self._ensure_not_safe_stop()
            if not self.execution_enabled or not self.config.get(
                    "qr_execution_authorized", False) or not self.config.get(
                        "supervised_commissioning_authorized", False):
                raise TaskManagerError("监督恢复持有工具未授权")
            slot_id = int(self._selected_tool()["slot_id"])
            result = self.ros.call_five_target_task(
                "confirm_held_tool", slot_id, execute=False,
                supervised_confirm=True)
            if int(result.get("holding_slot", -1)) != slot_id:
                raise TaskManagerError("恢复后未记录Tool{}".format(slot_id))
            if int(result.get("voltage_v", -1)) != 24:
                raise TaskManagerError("恢复持有状态未取得24V ACK")
            self.state_machine.transition(
                RobotState.TOOL_HOLD,
                "监督确认Tool{}仍保持吸合".format(slot_id))
            return result

    def cancel_target_execution(self) -> Dict[str, Any]:
        return self.ros.call_five_target_task(
            "cancel", 0, execute=False)

    def pick_tool(self) -> Dict[str, Any]:
        with self._operation_lock:
            self._ensure_not_safe_stop()
            if self.launches.mode is not None:
                raise TaskManagerError("请先停止Touch遥操作或力控节点，再执行工具取放")
            tool = self._selected_tool()
            self.state_machine.transition(RobotState.TOOL_PICK, "调用MoveIt取工具任务")
            try:
                if not self.execution_enabled:
                    result = self.ros.call_qr_task(
                        "preview", int(tool["slot_id"]), execute=False
                    )
                    self.state_machine.transition(
                        RobotState.TOOL_DETECT,
                        "仅完成规划预览；未执行取工具",
                    )
                    result["preview_only"] = True
                    return result
                result = self.ros.call_qr_task(
                    "pick", int(tool["slot_id"]), execute=True
                )
                if result.get("state") != "HOLDING":
                    raise TaskManagerError(
                        "QR task did not reach HOLDING: {}".format(result.get("state"))
                    )
                self.state_machine.transition(RobotState.TOOL_HOLD, "工具已取出并确认持有")
                return result
            except Exception as error:
                self._fail_safe(error, "取工具")
                raise

    def return_tool(self) -> Dict[str, Any]:
        with self._operation_lock:
            self._ensure_not_safe_stop()
            if self.launches.mode is not None:
                raise TaskManagerError("请先停止Touch遥操作或力控节点，再执行工具取放")
            tool = self._selected_tool()
            self.state_machine.transition(RobotState.TOOL_RETURN, "调用MoveIt放回工具任务")
            try:
                result = self.ros.call_qr_task(
                    "return", int(tool["slot_id"]), execute=self.execution_enabled
                )
                if not self.execution_enabled:
                    self.state_machine.transition(
                        RobotState.IDLE, "仅完成RETURN规划预览；未执行工具归还"
                    )
                    result["preview_only"] = True
                    return result
                if result.get("state") != "IDLE":
                    raise TaskManagerError(
                        "QR task did not return to IDLE: {}".format(result.get("state"))
                    )
                self.state_machine.transition(RobotState.IDLE, "工具已放回")
                return result
            except Exception as error:
                self._fail_safe(error, "放回工具")
                raise

    def enter_force_control(self) -> Dict[str, Any]:
        with self._operation_lock:
            self._ensure_not_safe_stop()
            if "/joint_control_server" in set(rosnode.get_node_names()):
                raise TaskManagerError(
                    "基础栈当前处于七轴真机控制模式；请切换基础栈模式后再进入力控"
                )
            if not self.execution_enabled:
                raise TaskManagerError(
                    "真实执行未授权，无法以enable_motion:=true进入垂直下压力控"
                )
            try:
                if not self.systems.status().get("base", {}).get("running", False):
                    raise TaskManagerError("请先在界面启动RM75基础栈")
                self.launches.start("force_control")
                self.ros.wait_for_node(
                    "/rm75_vertical_press", self.config["node_start_timeout_s"]
                )
                self.ros.wait_for_force_feedback(self.config["feedback_timeout_s"])
                self.state_machine.transition(
                    RobotState.FORCE_CONTROL, "垂直下压力控已就绪"
                )
                return {"mode": "force_control", "execution_enabled": self.execution_enabled}
            except Exception as error:
                self._fail_safe(error, "进入力控")
                raise

    def prepare_tool5_drill(self) -> Dict[str, Any]:
        """Start the existing vertical controller in dedicated, non-Touch mode."""
        with self._operation_lock:
            self._ensure_not_safe_stop()
            if not self.execution_enabled or not self.config.get(
                    "tool5_drill_authorized", False):
                raise TaskManagerError(
                    "Tool5力控钻进未授权；必须使用专用真机授权启动参数")
            if self.joint_adapter is not None and self.joint_adapter.active:
                raise TaskManagerError("七轴关节控制正在执行，拒绝Tool5钻进抢占")
            if self.launches.mode is not None:
                raise TaskManagerError("已有Touch或力控命令源，拒绝Tool5钻进")
            if not self.systems.status().get("base", {}).get("running", False):
                raise TaskManagerError("请先在界面启动RM75基础栈")
            try:
                self.launches.start("tool5_drill")
                self.ros.wait_for_node(
                    "/rm75_vertical_press", self.config["node_start_timeout_s"])
                self.ros.wait_for_force_feedback(self.config["feedback_timeout_s"])
                self.state_machine.transition(
                    RobotState.FORCE_CONTROL, "Tool5钻进控制器已就绪")
                return {"mode": "tool5_drill", "prepared": True}
            except Exception as error:
                self._fail_safe(error, "准备Tool5钻进")
                raise

    def engage_tool5_drill(self, depth_m: float) -> Dict[str, Any]:
        with self._operation_lock:
            self._ensure_not_safe_stop()
            if self.launches.mode != "tool5_drill":
                raise TaskManagerError("Tool5钻进控制器尚未准备")
            self.ros.start_tool5_drill(depth_m)
            return {"mode": "tool5_drill", "depth_m": float(depth_m)}

    def hold_tool5_drill(self) -> Dict[str, Any]:
        """Stop Z feed without stopping the controller or retracting the arm."""
        with self._operation_lock:
            if self.launches.mode != "tool5_drill":
                raise TaskManagerError("Tool5钻进控制器当前未由界面启动")
            self.ros.stop_tool5_drill_feed()
            return {"mode": "tool5_drill", "holding": True}

    def start_sampling(self) -> Dict[str, Any]:
        """Validate that the existing Touch-controlled sampler is engaged."""

        with self._operation_lock:
            self._ensure_not_safe_stop()
            if self.launches.mode != "force_control":
                raise TaskManagerError("垂直下压力控节点尚未启动")
            snapshot = self.ros.telemetry_snapshot()
            if not snapshot.get("teleop_active", False):
                raise TaskManagerError(
                    "垂直下压力控已就绪但尚未激活；灰键控制姿态，白键控制位置，双键锁定XY并沿Z下降"
                )
            mode = snapshot.get("control_mode")
            if mode == "SAFE_STOP":
                error = TaskManagerError("底层力控已进入SAFE_STOP")
                self._fail_safe(error, "开始采样")
                raise error
            LOGGER.info("采样状态确认：active=true, mode=%s", mode)
            return snapshot

    def stop_sampling(self) -> Dict[str, Any]:
        with self._operation_lock:
            if self.launches.mode != "force_control":
                raise TaskManagerError("垂直下压力控节点当前未由界面启动")
            if self.ros.telemetry_snapshot().get("teleop_active", False):
                raise TaskManagerError(
                    "请先松开Touch使active=false；需要立即停止时请使用急停"
                )
            try:
                self.launches.stop()
                self.state_machine.transition(
                    RobotState.IDLE, "力控采样节点已停止"
                )
                return {"stopped": True, "mode": "force_control"}
            except Exception as error:
                self._fail_safe(error, "停止采样")
                raise

    def set_voltage(self, voltage: int) -> Dict[str, Any]:
        with self._operation_lock:
            if not self.execution_enabled:
                raise TaskManagerError("真实执行未授权，拒绝改变工具电压")
            self._ensure_not_safe_stop()
            if voltage not in (0, 24):
                raise TaskManagerError("仅允许0V或24V")
            try:
                return self.ros.set_tool_voltage(voltage)
            except Exception as error:
                self._fail_safe(error, "设置工具电压")
                raise

    def emergency_stop(self) -> Dict[str, Any]:
        """Stop immediately; this method intentionally bypasses operation_lock."""

        self.state_machine.fail(RuntimeError("operator emergency stop"), "SAFE_STOP")
        errors = []
        result: Dict[str, Any] = {}
        if self.joint_adapter is not None:
            try:
                self.joint_adapter.cancel()
            except Exception as error:
                errors.append(str(error))
        try:
            result = self.ros.emergency_stop()
        except Exception as error:
            errors.append(str(error))
        try:
            snapshot = self.ros.telemetry_snapshot()
            if bool(snapshot.get("health", {}).get("qr", False)):
                self.ros.call_five_target_task("stop", 0, execute=False)
        except Exception as error:
            errors.append(str(error))
        try:
            self.launches.stop()
        except Exception as error:
            errors.append(str(error))
        self.state_machine.fail(
            RuntimeError("; ".join(errors) if errors else "operator emergency stop"),
            "SAFE_STOP",
        )
        if errors:
            raise TaskManagerError("停止请求存在未确认项：{}".format("; ".join(errors)))
        return result

    def recover(self) -> Dict[str, Any]:
        """Clear controller/task errors without generating recovery motion."""

        with self._operation_lock:
            self.state_machine.transition(RobotState.RECOVERY, "开始受控恢复")
            try:
                self.launches.stop()
                clear_result = self.ros.clear_system_error()
                snapshot = self.ros.telemetry_snapshot()
                qr_online = bool(snapshot.get("health", {}).get("qr", False))
                if not qr_online:
                    status = {"state": "NOT_RUNNING", "skipped": True}
                    self.state_machine.transition(RobotState.IDLE, "控制器恢复完成（QR未启动）")
                    return {"driver": clear_result, "task": status}
                # ``health.qr`` tracks /rm75_five_target_server.  It must not
                # be used as evidence that the retired single-tool service
                # /qr_task/command is available.  Unified QR and saved-pose
                # control paths intentionally start only the five-target
                # server, so querying the legacy service here made every
                # recovery wait for a timeout and re-latch SAFE_STOP.
                status = self.ros.call_five_target_task(
                    "status", 0, execute=False)
                holding_slot = status.get("holding_slot")
                target = RobotState.IDLE
                if holding_slot is not None:
                    slot_id = int(holding_slot)
                    matches = [
                        name for name, tool in self.tools.items()
                        if int(tool["slot_id"]) == slot_id
                    ]
                    if len(matches) != 1:
                        raise TaskManagerError(
                            "五目标服务报告未知持有槽位{}".format(slot_id))
                    self.selected_tool = matches[0]
                    target = RobotState.TOOL_HOLD
                self.state_machine.transition(target, "恢复检查完成")
                return {"driver": clear_result, "task": status}
            except Exception as error:
                self.state_machine.fail(error, "恢复失败")
                raise

    def shutdown(self) -> None:
        if self.joint_adapter is not None:
            self.joint_adapter.cancel()
        self.launches.stop()
        self.systems.shutdown()

    def _selected_tool(self) -> Dict[str, Any]:
        if not self.selected_tool:
            raise TaskManagerError("尚未选择或识别工具")
        return self.tools[self.selected_tool]

    def _ensure_not_safe_stop(self) -> None:
        if self.state_machine.state == RobotState.SAFE_STOP:
            raise TaskManagerError("SAFE_STOP已锁存，请先执行恢复检查")

    def _fail_safe(self, error: BaseException, context: str) -> None:
        LOGGER.error("%s失败，进入SAFE_STOP：%s", context, error)
        self.state_machine.fail(error, context)
        try:
            self.ros.emergency_stop()
        except Exception as stop_error:
            LOGGER.error("SAFE_STOP停止ACK失败：%s", stop_error)
        try:
            self.launches.stop()
        except Exception as launch_error:
            LOGGER.error("停止控制launch失败：%s", launch_error)
        self.state_machine.fail(error, context)
