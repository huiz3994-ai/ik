# 手动示教轨迹验收记录（2026-10-28）

## 范围与实际结果

本次为软件与 SIMULATION 验收，没有启动真实机械臂运动。真实路径仍使用既有
`TaskManager → JointControlAdapter → /rm75_sampling_gui/follow_joint_trajectory`，
没有新增第二个 JointPos 发布者。MoveIt 相关单元测试使用可控碰撞服务替身；SIMULATION
不代表已经完成现场碰撞场景、物理跟踪误差、制动距离或停止 ACK 的真机验收。

| 检查 | 本轮结果 |
|---|---|
| 全包 pytest，排除单独注册的 ROS transport rostest | 179 passed，12.65 s |
| `catkin build rm75_sampling_gui`，Release，含依赖 | 15 packages succeeded，0 failed |
| `catkin run_tests` + `catkin_test_results` | 90 tests，0 errors，0 failures，0 skipped |
| React TypeScript 检查 | 通过 |
| Vite 生产构建 | 通过 |
| 统一入口 SIMULATION 黑盒脚本 | PASS |
| 实际构建页面操作 | 单击记录、生成、保存保留 READY、参数改动禁止旧轨迹执行均通过 |

构建警告：六个依赖包输出 CMake 策略/旧版本或 Gazebo Classic 弃用提示；Vite 提示
charts/three chunk 超过 500 kB。这些不属于测试失败，本轮不改动依赖配置。
pytest 与 catkin 部分用例重叠，不能把两个数量相加当成独立测试数。

## 恢复审查与修复

1. **重复 ID 覆盖资源**：先复现创建同 ID 会丢失原示教点，再改为已 fsync 的临时文件通过
   不覆盖的原子链接发布。多个 repository 实例同时创建仅允许一个成功，冲突返回 HTTP 409，
   原文件字节保持不变。覆盖更新仍通过明确的保存/修改接口。
2. **取消丢失**：先复现校验中取消后仍发送 Action，再增加每次运行独立的取消事件与 adapter
   取消代次。覆盖 worker 尚未进入、校验中、Action 等待中和执行中。提交前重新读取起点反馈；
   不再清除已接受的停止请求。执行中禁止覆盖、删除及阻塞停止路径的重新生成。
3. **回返进度跳段**：用更靠近实际位置的未来回返点复现全局最近点跳段；改为沿当前段的
   第一个局部最近点推进，结合已到达执行时间限制匹配上界。位置歧义时进度保守保留，最终
   完成仍依赖 Action 成功和末端关节误差确认。
4. **执行插值退化**：受控 Action 服务原先忽略导数字段。现消费完整 position/velocity/
   acceleration 重建五次 Hermite；测试在非采样时刻与原多项式一致，并保留关键点通过速度。
   预检覆盖重建曲线的位置/速度极值和密集碰撞检查。旧的仅位置目标保持兼容。
5. **仿真停止与保存**：仿真使用同一五次曲线，急停后保持 SAFE_STOP；取消和租约丢失冻结
   模拟目标。相同参数保存保留已生成轨迹；页面参数未重新生成时禁用执行。

首批新增回归在修复前为 5 failed / 24 passed；后续执行插值、保存和急停回归在修复前为
4 failed / 88 passed。最终全包 179 项通过。

## 覆盖的需求

- 连续单击记录 20 个点，与七轴实际反馈、TF 四元数及源时间戳一致；拒绝陈旧/缺失反馈。
- 点位 ID、序号、删除、上移下移、清空和参数修改失效。
- 原子 JSON CRUD、损坏文件隔离、非法 ID/路径穿越拒绝、重启读取恢复。
- 每个原始向量精确写入对应轨迹结点，C2 连续；单调段非零通过速度、反转轴零通过速度。
- 密集速度/加速度限制、关节限位、碰撞拒绝、标准 ROS JointTrajectory 字段与顺序。
- ID 调用、反馈进度、取消、起点误差、重复执行和租约丢失。
- 原有笛卡尔十二方向点动、Touch、力控、采样硬件、工具任务、SAFE_STOP 与相机 JPEG 回归。

## 可复现命令

在项目根目录运行：

```bash
docker exec rm75_noetic bash -lc 'source /opt/ros/noetic/setup.bash; source /root/rm75_ws/devel/setup.bash; /root/rm75_ws/.venv-rm75-web/bin/python -m pytest /root/rm75_ws/src/rm75_sampling_gui/test --ignore=/root/rm75_ws/src/rm75_sampling_gui/test/test_monitor_transport.py -q'
docker exec rm75_noetic bash -lc 'source /opt/ros/noetic/setup.bash; cd /root/rm75_ws; catkin build rm75_sampling_gui --no-status -j2 --cmake-args -DCMAKE_BUILD_TYPE=Release'
docker exec rm75_noetic bash -lc 'source /opt/ros/noetic/setup.bash; source /root/rm75_ws/devel/setup.bash; cd /root/rm75_ws; catkin run_tests rm75_sampling_gui --no-deps --no-status -j2 && catkin_test_results build/rm75_sampling_gui/test_results'
npm --prefix rm75_ws/src/rm75_sampling_gui/web/frontend run typecheck
npm --prefix rm75_ws/src/rm75_sampling_gui/web/frontend run build
```

`test_monitor_transport.py` 由 CMake 注册为 rostest，需要 ROS 节点初始化，不能直接作为
普通 pytest 文件运行。上述 catkin 命令包含它。

仿真入口与黑盒验收在两个终端运行，测试后 Ctrl+C 正常停止入口：

```bash
RM75_WEB_PORT=8095 bash rm75_ws/src/rm75_sampling_gui/scripts/start_sampling_web.sh --mode simulation
docker exec rm75_noetic bash -lc 'source /opt/ros/noetic/setup.bash; source /root/rm75_ws/devel/setup.bash; /root/rm75_ws/.venv-rm75-web/bin/python /root/rm75_ws/src/rm75_sampling_gui/scripts/validate_web_simulation.py --url http://127.0.0.1:8095'
```

注意入口省略 `--mode simulation` 会使用现场统一控制配置，本次验收始终显式指定仿真。

## 资源与交付边界

### 功能文件清单

以下路径相对 `rm75_ws/src/rm75_sampling_gui/`，标注新增的文件是相对本功能开始前版本：

| 文件 | 内容 |
|---|---|
| `trajectory/__init__.py`、`models.py`、`interpolation.py`、`repository.py`、`service.py`（新增） | 版本化模型、严格过点五次插值、原子 JSON、统一资源执行服务 |
| `ros/monitoring.py` | TF 四元数与源时间戳 |
| `control/joint_control_adapter.py` | 密集轨迹验证、ROS 消息、反馈进度与取消 |
| `task/task_manager.py` | 接入既有互斥执行链 |
| `scripts/joint_control_server.py` | 受控服务消费五次导数，曲线预检 |
| `web/backend/app.py`、`services.py` | FastAPI 资源接口与 SIMULATION |
| `web/frontend/src/App.tsx`、`types.ts`、`store.ts`、`styles.css` | 路径示教页面、数据契约、状态和响应式布局 |
| `web/frontend/src/trajectory-contract.check.ts`（新增） | TypeScript 契约检查 |
| `test/test_waypoint_trajectory.py`（新增） | 模型、算法、持久化、执行与竞态回归 |
| `test/test_web_backend.py`、`test/test_joint_control_server.py` | Web/API、仿真与 ROS 执行服务回归 |
| `scripts/validate_web_simulation.py` | 统一入口黑盒验收 |
| `config/gui.yaml`、`setup.py` | 存储目录/加速度配置与 Python 子包安装 |
| `README.md`、`docs/web/API.md`、本文（新增） | 使用、接口及验收说明 |

项目根目录另更新 `PROJECT_FOR_CHATGPT.md`，新增设计与实施计划
`docs/superpowers/specs/2026-10-28-rm75-waypoint-trajectory-design.md` 和
`docs/superpowers/plans/2026-10-28-rm75-waypoint-trajectory.md`。

JSON 字段包含 schema/version、稳定 ID、名称、机器人/关节/坐标系/工具、创建及更新时间、
原始 waypoints、trajectory_config、密集 trajectory、trajectory_status。默认目录为运行
后端用户的 `~/.ros/rm75_sampling_gui/trajectories/`；当前容器中对应
`/root/.ros/rm75_sampling_gui/trajectories/`，可用 `trajectory_library.directory` 调整。

仓库分支为 `feature/rm75-waypoint-trajectory`。上轮功能提交 `62bd595` 已保留，本轮审查修复
另作提交，以普通非强制推送发布到已配置 origin。构建缓存和临时日志排除；原有 rm_driver
两个源文件及 catkin 缓存 package.xml 的用户改动保留且不提交。
