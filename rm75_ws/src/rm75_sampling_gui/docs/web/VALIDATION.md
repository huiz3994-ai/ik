# Web 重构验证记录（更新至 2026-10-28）

## 2026-10-28 Link7 笛卡尔点动（仅仿真/自动化）

- `catkin build rm75_motion_planning rm75_sampling_gui --no-status -j2`：连同依赖共 17 个包
  Release 构建成功，0 warning、0 failed。新增 `rm75_cartesian_jog_server` 已实际编译、链接。
- 受影响包 `catkin run_tests` 全部通过：`rm75_motion_planning` 66、
  `rm75_sampling_gui` 88、`rm75_touch_teleop` 57、`rm75_vertical_press` 12、
  `rm75_qr_task` 30；共 253 项，0 errors、0 failures、0 skipped。末端方向专项 C++
  测试覆盖 6 个基坐标平移和 6 个 Link7 局部旋转。
- 容器 Web venv 中 `pytest -q test/test_web_backend.py`：50 passed。覆盖固定方向白名单、
  独占租约、无租约/SAFE_STOP 拒绝、12 个仿真方向、10 mm/5° 上限、租约丢失停止，以及
  SIMULATION 不调用 ROS 运动输出；另验证执行器退出只补发一次 ACK-checked 驱动停止，停止
  失败仍保持 SAFE_STOP。IK 失败、关节限位、跟踪误差、下一周期预测和碰撞拒绝由
  `rm75_motion_planning` 的 ContinuousIk/Validator/RuntimeGuard/Collision 测试覆盖。
- `validate_web_simulation.py --url http://127.0.0.1:8093`：PASS。真实 HTTP/WebSocket
  黑盒依次开始/停止十二个方向，并验证 WebSocket 直接断开后租约看门狗冻结内存植物且记录
  `lease expired`；同时回归七轴、Z 力位/Touch 触觉、采样硬件、SAFE_STOP、二维码、五工具和
  相机 JPEG。仿真启动参数保持所有真实执行授权为 false。
- 前端 `npm run build`：TypeScript 和 Vite 生产构建成功（2780 modules transformed）。使用
  768px 可视浏览器检查侧栏入口、两个圆形交叉控制盘、弧形旋转图标、Link7 实际/目标位姿、
  状态和停止原因；取得仿真控制租约后点击 X+，页面返回 STOPPED 且目标 X 增加 10 mm。
- 仿真初版在状态刷新中重复运行 SciPy IK，曾使 650 ms 浏览器租约心跳饥饿。已改为 START
  前对最大目标求解一次 IK，运行中只插值该已验证关节解；修复后十二方向与断联黑盒通过。
- 本轮没有连接控制器、没有启动真实驱动、没有发布 `/rm_driver/JointPos`，也没有执行工具
  电源、电机或 commissioning。本节结果不构成真机方向、停止距离或碰撞场景验收。

## 2026-10-25 七轴实机冒烟测试

- 控制器 `192.168.1.18:8080` 连通，关节与六维力反馈均约 200 Hz；驱动、MoveIt、
  `joint_control_server`、TF 和 WebSocket 状态正常。
- 真实当前姿态执行通过：耗时 1.241 s，最大跟踪误差 0.000401 rad，状态返回 IDLE。
- J7 `+0.5°` 往返通过：实际外移约 0.486°；外移最大误差 0.000384 rad，返回最大误差
  0.000925 rad；没有 SAFE_STOP 或活动诊断故障。
- 真实当前姿态 FK→IK 自洽通过：位置误差约 6.7e-8 m、姿态误差约 2.9e-8 rad，七轴
  回算最大差约 7.45e-5 rad。
- 修复统一启动器默认 RS-485 脚本仍指向已删除桌面交付目录的问题，改用工作区受审副本
  `.rm75_hardware/modbus.py`。
- 将 `joint_control` 与采样硬件授权解耦：七轴实机模式不再因未插 RS-485/DM 电机而报
  无关串口故障；Tool5 模式仍保留采样硬件授权。
- MoveIt 启动仍报告 `rm_group/follow_joint_trajectory` action 未连接；本次受控七轴路径走
  独立 `joint_control_server` 并已成功。该警告对二维码/trajectory bridge 路径的影响需在
  对应模式单独验收。

## 已执行

- 前端 `npm run build`：TypeScript 与 Vite 生产构建成功。
- Web 安全/API 测试：9/9 通过，覆盖独占租约、WS 绑定、过期、伪造 token、离线预览不创建
  控制会话、命令在 WS 心跳前被拒绝。
- 原 `rm75_sampling_gui` catkin/rostest 回归：`65 tests, 0 errors, 0 failures, 0 skipped`；
  与 5 项 Web 专项测试合计 70 项通过。
- OFFLINE 端到端启动：FastAPI、静态前端、WebSocket、REST、8 个厂商 STL 路由均正常。
- 模型接口核验：`RM75-6FB`、7 个关节、8 个 visual；joint7 origin Z=`0.1612`；最后网格
  为厂商 `Link7_6fb.STL`。
- 浏览器人工检查：操作台、65/35（桌面）/纵向（小屏）机器人页、实际/目标双模型、七轴
  控件和 OFFLINE 禁止控制标识可见。
- 正式 `start_sampling_web.sh` 观察模式启动成功：自动创建 ROS Master，仅出现
  `/rm75_sampling_web` 与 `/rosout`；Ctrl+C 后 Web、roslaunch、rosmaster 均退出，无孤儿进程。
- 新增纯内存 SIMULATION 模式和可重复黑盒验收脚本。Web 专项测试增至 6/6，与原 65 项
  ROS 测试合计 71 项。
- `RM75_CONTROL_PATH=simulation RM75_WEB_PORT=8092` 完整启动成功；浏览器确认顶栏
  `SIMULATION / 无物理输出`、独占租约、工具 24V 状态、厂商 STL 双模型和实时反馈。
- `validate_web_simulation.py` 黑盒通过：健康标志、WebSocket 心跳绑定、七轴目标跟踪、
  工具电压、SAFE_STOP 锁存/409 拒绝/恢复、合成相机 JPEG。
- 仿真运行期间 ROS Master 仅有 `/rosout`；`/rm_driver/JointPos` 与
  `/rm75_sampling_gui/joint_control_heartbeat` 均不存在。Ctrl+C 后 8092 端口、Web 和
  roslaunch 均退出，无孤儿进程。

## 安全结论

- `/joints/preview` 无需控制租约，代码只调用 `RM75RobotModel.forward_kinematics`。
- 执行 endpoint 没有 JointPos publisher；只调用现有 `TaskManager.execute_joint_target`。
- 浏览器命令要求独占、WS 已绑定且未过期的 session token。
- 浏览器断联后 server 不再发布既有 GUI heartbeat；没有扩大 0.35 s ROS watchdog。
- SAFE_STOP 状态来自原状态机；恢复调用原 `TaskManager.recover`。
- 服务硬限制 `127.0.0.1`，没有开放公网或局域网。
- SIMULATION 不连接 `ROSInterface`，命令在内存模拟器终止；启动脚本强制真实执行、QR、
  joint_control、trajectory 和 supervised 授权全部为 false。

## 2026-10-20 真机零运动验收

- 有线主机 `192.168.1.100` 到控制器 `192.168.1.18`：3/3 ping，无丢包；TCP 8080 可连接。
- 使用 `RM75_CONTROL_PATH=observe`，五项执行授权全部为 false。基础栈启动后节点为
  `rm_driver`、`robot_state_publisher`、`move_group`、`trajectory_bridge`；bridge 的
  `execute=false`，`/rm_driver/JointPos` 虽有该节点注册为 publisher，但连续监测无消息。
- `/joint_states` 实测约 200 Hz，七轴使能均为 true，关节电压 22–23 V，温度 29–34 ℃；
  5 秒静止窗口各轴跨度约 0.000052–0.000087 rad。当前姿态经 `/check_state_validity`
  判定 valid=true、contacts=0，`base_link -> Link7` TF 与 Web TCP 一致。
- 六维力数据持续新鲜；5 秒样本 Fz 均值约 -2.09 N、标准差约 0.085 N。此值未做现场去皮，
  仅作为通信稳定性证据。
- D435i RGB 实测 424×240、rgb8、约 5.99 Hz，Web JPEG 424×240 可解码；驱动报告设备通过
  USB 2.1 连接，因此只确认当前 RGB 6 Hz 能力，不声称 USB 3/RGB-D 验收。
- 观察模式在实际姿态目标上点击执行，被
  `JOINT_CONTROL_EXECUTION_AUTHORIZED=false` 以 HTTP 409 拒绝；拒绝后 `JointPos` 无消息，
  状态仍为 IDLE。
- 发现并修复厂商 position-only `JointState` 被 API 错标为
  `fresh.joint_velocities=true`：现在 `joint_velocity_source=unavailable`、速度字典为空时
  明确返回 false，并增加回归断言。修复后 65 项 ROS 测试与 6 项 Web 测试全部通过。
- 首轮驱动曾在启动约 50 秒后打印一组 `Read_UDP_date IS error` 和 `Connect IS failed`，
  同期关节姿态发生明显变化，但 ROS 命令话题均无消息；第二次启动越过同一时间窗口未复现。
  在确认是否为现场示教器/人工移动及清空机械臂工作区之前，没有进行新的运动测试。

## 未声称完成

本轮没有执行新的真机运动、工具电压、Touch 或力控动作。Web 浏览器断联到真实制动距离、
Action cancel 的最坏停止时间、完整七轴包络、持载、真实环境 PlanningScene、实体急停链、
D435i USB 3 RGB-D、工具几何/抓取仍需现场验收。OFFLINE 页面和 Mock/API 测试不是真机验收。

## 五工具配置与取物（2026-10-21）

- React/TypeScript 生产构建通过；浏览器检查确认五工具页、7 个配置输入、实时位姿、0–6
  步状态和安全提示正常渲染。
- SIMULATION 黑盒验收通过：完整读取 5 组持久化配置，Tool1 Dry Run、执行、步骤 6/6，
  并验证没有物体在位传感器时只返回 `COMPLETED_UNVERIFIED`。
- `rm75_qr_task`：30 tests，0 errors/failures；`rm75_sampling_gui`：71 tests，0
  errors/failures；两个包 Release 构建成功。
- 未自动启动或移动真实机械臂。真机仍需先做断使能规划、空载低速单工具、物理急停和
  末端负载验收；当前测试不构成抓取成功或整机安全认证。
- 上位机已将“二维码取物”和“自主写点取物”拆成两个独立导航入口。二维码页使用原有
  ID 0/标定槽位链路并补齐执行、归还和取消；自主写点页只使用持久化绝对位姿。浏览器
  仿真检查确认两个入口、各自目标来源说明和控制按钮均正常显示。
