# Link7 笛卡尔点动

## 坐标约定

- 末端参考是厂商 RM75-6FB 模型的 `Link7`，不是经过标定的工具 TCP。
- `x_plus/x_minus`、`y_plus/y_minus`、`z_plus/z_minus` 在 `base_link` 中平移。
- `rx_plus/rx_minus`、`ry_plus/ry_minus`、`rz_plus/rz_minus` 在当前 `Link7`
  局部坐标系中旋转，即 `R_target = R_start * R_local(axis, angle)`。
- 页面显示 XYZ（mm）和固定轴 XYZ RPY（°）；ROS 执行与 API 内部仍使用 SI 单位。

## 调用链

`React hold button -> FastAPI allowlisted API -> WebRuntime exclusive path ->
ROS Trigger service -> rm75_cartesian_jog_server -> CommandSourceManager ->
GuardedCommandGateway/RuntimeGuard -> /rm_driver/JointPos`

`rm75_cartesian_jog_server` 是专用、连续的 50 Hz 执行器，不是把一次性 commissioning
命令重复包装成点动。开始时它从新鲜关节反馈计算 `base_link -> Link7`，生成单次最大
10 mm 或 5° 的完整 SE(3) 路径，进行连续 IK、0.05/0.10/0.50 SI 动态限制、关节裕量和
MoveIt/FCL 检查，然后在按住期间逐周期通过安全网关输出。每周期再次检查当前关节、下一
目标、速度预测、碰撞、反馈时效、跟踪误差、力/力矩和驱动状态。

执行器只在 `cartesian_jog_execution_authorized=true` 且
`supervised_commissioning_authorized=true` 时建立唯一 JointPos 发布者。默认值均为 false。
启动前还要求 `/check_state_validity` 可调用，并拒绝 joint server、Touch、垂直力控、
trajectory bridge、commissioning 等冲突节点。Web 控制路径切换不会抢占外部节点。

## API

开始点动：

```http
POST /api/v1/cartesian/jog/start
X-RM75-Control-Session: <opaque lease>
Content-Type: application/json

{"direction":"x_plus"}
```

`direction` 只能是：`x_plus, x_minus, y_plus, y_minus, z_plus, z_minus,
rx_plus, rx_minus, ry_plus, ry_minus, rz_plus, rz_minus`。

停止和状态：

```http
POST /api/v1/cartesian/jog/stop
GET  /api/v1/cartesian/jog/status
```

状态包含 `state`、`direction`、`actual_pose`、`target_pose`、`progress`、
`stop_reason`、坐标系和实际后端限制。浏览器不能提交速度、距离、ROS 名称或 shell 参数。

## 停止语义

- pointer up、pointer cancel、pointer capture 丢失、窗口失焦、页面隐藏、组件卸载和主动释放
  控制权都会请求停止；键盘 Space/Enter 采用 keydown 开始、keyup 停止。
- WebSocket 断开或租约过期后，后端不再发布 100 ms 租约派生心跳；ROS 执行器的 350 ms
  看门狗触发停止。
- 松开或看门狗停止会停止后续 JointPos 流，调用既有 `/rm_driver/Emergency_Stop`（驱动
  `Move_Stop_Cmd`），等待新的 `/rm_driver/Set_Arm_Stop_Result=true`，并以新鲜关节反馈连续
  5 个 50 ms 样本确认静止。只发送停止请求不会返回成功。
- 达到 10 mm/5° 上限也走相同 ACK+静止确认，状态保持 `LIMIT_REACHED`，必须松开后才可
  再次开始。停止超时进入 `SAFE_STOP`，不会自动恢复。
- Web 后端检测到活动中的笛卡尔执行器/基础栈退出时锁存 `SAFE_STOP`，并通过现有
  `ROSInterface.emergency_stop()` 做一次新鲜 ACK 校验的后备停止；失败记录
  `STOP_UNCONFIRMED`，不会把“已发送”报告成“已停止”。
- 仿真不连接 ROS，也不创建 `/rm_driver/JointPos` 发布者；其停止确认只代表内存植物冻结。
  仿真在 START 前只对本次最大目标求一次 IK，随后在已验证的起止关节解之间插值，避免状态
  刷新计算阻塞 650 ms 浏览器租约心跳；显示位姿始终由模拟关节反馈重新做 FK 得到。

## 真机验收边界

自动化和内存仿真不构成真机验收。真机阶段必须另行授权，先断使能/无物理输出核验节点与
服务，再在清空工作区、实体急停就绪、低速空载条件下逐轴验证方向、制动距离、ACK 延迟、
反馈/网络断联、碰撞场景、关节限位和持载行为。
