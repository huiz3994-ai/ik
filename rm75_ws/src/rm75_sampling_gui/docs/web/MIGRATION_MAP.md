# PyQt → Web 迁移映射

| 原 PyQt 组件/操作 | React 页面/组件 | 后端接口/复用服务 | ROS 数据源或危险边界 |
|---|---|---|---|
| `MainWindow` 操作台 | `Dashboard` + 全局状态栏/事件流 | `GET /state`、`POST /system/{name}/{start|stop}`；`SystemLaunchManager` | 受管 `base_real`、D435i、QR launch；不接受任意 shell |
| `StatusWidget` | 系统态势卡、生命周期列表 | WebSocket `state` | ROS Master XML-RPC、节点图、`/joint_states`、TF |
| `Robot3DWidget` | `RobotTwin` (Three/R3F) | `GET /model`、只读 STL route | 厂商 6FB xacro、`Link7_6fb.STL`；Three 不做权威碰撞 |
| `JointControlWidget` | `JointPanel` | `POST /joints/preview`、`POST /joints/execute`、`cancel_joints` | **危险：**只经 `JointControlAdapter` 和既有 Action；滑块不发命令 |
| GUI 定时心跳 | 浏览器独占控制租约 | WebSocket `heartbeat` → `ControlSessionManager` | **危险：**租约有效才发布原 joint server 心跳；断联 fail-safe |
| `ForceMonitorWidget` | 六维数值、闭环卡、Recharts Fz | WebSocket 状态、force task commands | Six_Force 与 vertical_press 只读反馈；算法仍在 ROS 节点 |
| `VisionWidget` | RGB 面板、工具选择、位姿卡 | `GET /vision/frame.jpg`、受限 command allowlist | D435i 424×240@6 RGB、既有 Mark/PnP；不在前端做 PnP |
| `DiagnosticWidget` | 活动故障、事件时间线、恢复卡 | derived diagnostics、`recover` | **危险：**SAFE_STOP 来自同一 `StateMachine`，恢复调用既有流程 |
| `ParameterWidget` | 只读 YAML 折叠视图 | `GET /parameters` | 不写 rosparam，不修改运行中阈值 |
| Qt 日志文本框 | 全局事件流 | 有界 `EventBuffer` | 复用 Python/ROS 应用日志 |
| `ExperimentLogger` | 实验记录列表/API | `GET /experiments`；原 logger 10 Hz sample | 原 CSV/task.log/config.yaml 语义不变 |
| Qt `closeEvent` | WS disconnect/lease expiry | session release + ROS heartbeat停止 | **危险：**不伪造 GUI 心跳，不扩大 joint server 超时 |

涉及真机运动、安全停止或授权的行已用“危险”标记。旧 PyQt 文件和入口保持原样，作为
回退方案；公共业务逻辑没有复制成第二套控制实现。

最新现场配置为“单公共 AprilTag ID 0 引导五个工具目标”，因此 Web 页面按当前
`tools.yaml` 显示公共 ID 0；代码同时能显示旧五标记状态，但不会把历史 ID 映射覆盖当前配置。
