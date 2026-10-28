# Web API 与 WebSocket

默认基址：`http://127.0.0.1:8088`。交互式 OpenAPI：`/api/docs`。

`GET /api/v1/health` 和状态快照均包含 `offline`、`simulation` 标志。仿真模式沿用完全
相同的 API 合约，但所有写操作只作用于内存模拟状态。

## REST

| 方法 | 路径 | 语义 |
|---|---|---|
| GET | `/api/v1/health` | Web/ROS Master 基本健康状态 |
| GET | `/api/v1/state` | 单次完整状态快照，含 `timestamp`、`sequence`、fresh/age |
| GET | `/api/v1/model` | 厂商 xacro 解析出的关节 origin/axis/limit 和 mesh 清单 |
| GET | `/api/v1/model/mesh/{index}` | 仅允许模型清单中的 STL，不接受路径参数 |
| POST | `/api/v1/joints/preview` | 离线 FK；没有控制会话也可用，永不发 ROS 命令 |
| POST | `/api/v1/control/session` | 申请唯一控制租约 |
| DELETE | `/api/v1/control/session` | 主动释放租约 |
| PUT | `/api/v1/control/path` | 在全部控制子系统停止时选择唯一活动控制路径；不会启动运动 |
| POST | `/api/v1/joints/execute` | 调用现有自动检查+执行流程 |
| GET | `/api/v1/trajectories` | 查询已保存的 `RM75Trajectory` 资源与损坏文件扫描错误 |
| POST | `/api/v1/trajectories` | 创建并原子保存稳定 ID 的轨迹资源；需要租约 |
| GET/PUT/DELETE | `/api/v1/trajectories/{id}` | 加载、修改或删除轨迹；修改/删除需要租约 |
| POST | `/api/v1/trajectories/{id}/waypoints` | 从新鲜 `/joint_states` 与 `base_link -> Link7` TF 记录一个点 |
| DELETE | `/api/v1/trajectories/{id}/waypoints[/{waypoint_id}]` | 清空或删除示教点；需要租约 |
| POST | `/api/v1/trajectories/{id}/waypoints/{waypoint_id}/move` | 按 `up/down` 调整点位顺序；需要租约 |
| POST | `/api/v1/trajectories/{id}/generate` | 生成严格过点的分段五次轨迹并检查整轨迹限位/碰撞 |
| POST | `/api/v1/trajectories/{id}/execute` | 按稳定 ID 进入既有七轴受控执行链 |
| GET/POST | `/api/v1/trajectories/execution/{status|stop}` | 查询实际反馈进度或停止当前轨迹；停止需要租约 |
| POST | `/api/v1/cartesian/jog/start` | 以固定 12 方向枚举开始 Link7 受控点动；需要租约 |
| POST | `/api/v1/cartesian/jog/stop` | 请求停止并等待驱动 ACK 与实际静止确认；需要租约 |
| GET | `/api/v1/cartesian/jog/status` | 点动状态、方向、实际/目标位姿、限制和停止原因 |
| GET | `/api/v1/tools` | 读取五个持久化绝对 Link7 取物位姿 |
| PUT | `/api/v1/tools/{1..5}` | 原子保存 X/Y/Z、Rx/Ry/Rz 和 Lift Height；需要控制租约 |
| POST | `/api/v1/tools/{1..5}/dry-run` | 仅规划预抓取、严格基坐标 Z 下降/抬升完整路径，不运动 |
| POST | `/api/v1/tools/{1..5}/execute` | 仅执行同一配置、同一起点且已通过的 Dry Run 计划 |
| POST | `/api/v1/system/{base|camera|qr}/{start|stop}` | 受管生命周期操作 |
| POST | `/api/v1/commands/{allowlisted-name}` | 急停、恢复、取消、Touch、力控、识别、工具等白名单操作 |
| GET | `/api/v1/vision/frame.jpg` | 最新 RGB 帧；无新鲜帧返回 503 |
| GET | `/api/v1/parameters` | 只读 YAML 参数 |
| GET | `/api/v1/experiments` | 最近实验目录元数据 |

除读取和离线预览外，命令请求必须携带 `X-RM75-Control-Session`。后端不提供“发布任意
topic”“调用任意 service”“启动任意 launch”或 shell API。

轨迹资源默认保存在 `~/.ros/rm75_sampling_gui/trajectories/`，可通过
`trajectory_library.directory` 调整。文件名只使用经过校验的稳定 `trajectory_id`，保存采用
同目录临时文件、`fsync` 和原子替换。示教点或速度/加速度参数变化会清空旧密集轨迹并标记
`STALE`，必须重新生成后才允许执行。

笛卡尔点动的平移参考 `base_link`，旋转参考当前 `Link7` 局部轴，末端参考明确为
`Link7` 而不是已标定工具 TCP。浏览器只提交固定方向，不提交距离、速度、topic 或 service；
后端/ROS 执行器硬限制单次 10 mm/5°、Link7 线速度 0.01 m/s 和关节
0.05 rad/s、0.10 rad/s²、0.50 rad/s³。完整约定见 [CARTESIAN_JOG.md](CARTESIAN_JOG.md)。

五工具位置单位为米、姿态为固定轴 XYZ RPY（弧度），参考系为 `base_link`，目标为
`Link7`。执行端会再次读取持久化配置；配置改变或机械臂在 Dry Run 后移动都会使计划失效。
状态快照中的 `configured_pick_status` 给出 0–6 步、失败原因和最终确认能力。当前末端接口
只能确认 24V 命令的新 ACK，没有独立物体在位传感器，因此成功运动的最终状态是
`COMPLETED_UNVERIFIED`，不会报告“抓取成功”。

Z 轴力位触觉入口使用白名单命令 `start_z_force_teleop` 与
`stop_z_force_teleop`。前者只在 `touch_z_force_authorized=true`、真实执行已授权、基础栈
已启动、六维力新鲜且不存在 `/trajectory_bridge`、`/joint_control_server` 或独立力控节点时
接受。状态快照通过 `control_launch_mode=z_force_teleop`、`haptic_enabled`、
`haptic_force_fresh` 和 `haptic_command{x,y,z}` 回报真实节点状态；这些字段不是浏览器自行推断。

## WebSocket

路径 `/api/v1/ws`。服务端以 10 Hz 发送：

```json
{"type":"state","data":{"sequence":42,"timestamp":1792414207.1,"fresh":{"joints":true}}}
```

取得控制租约后，浏览器每 200 ms 发送：

```json
{"type":"heartbeat","session_id":"opaque-token"}
```

服务端只对绑定该 socket 的 token 返回 `heartbeat_ack`。重连不会恢复旧 token 的执行
授权；页面重新申请并绑定租约。高频消息采取“最新状态覆盖”，不累计历史队列；Fz 图只在
浏览器保留固定 300 点。
