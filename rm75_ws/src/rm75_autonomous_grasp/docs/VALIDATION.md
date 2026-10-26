# 验收记录

测试环境：2026-10-02，现有 `rm75_noetic` Docker 容器，ROS1 Noetic、MoveIt 1.1.16、Gazebo 11，Xvfb / 软件渲染深度相机。没有连接真机。下列数字为本次运行结果，不是多次统计的生产指标。

## 构建与回归

- 两个新增包及依赖共 8 包构建成功；最终构建见 [build_final.log](validation/raw/build_final.log)。
- 6 项视觉 gtest：毫米/米深度、无效深度、畸变反投影、颜色定位、AprilTag 等输入契约。
- 1 项 FCL gtest：端点无碰撞但中间穿障，以及三次控制器插值越障均被拒绝；移除障碍后通过。
- 1 项真机轨迹桥接 rostest：模拟驱动验证重排关节、无效轨迹拒绝、取消/停止确认、反馈过期失败。此项没有真实电机。
- 原始 XML 保存在 [unit](validation/unit/gtest-test_depth.xml)，日志见 [tests_final.log](validation/raw/tests_final.log)。catkin 汇总可能重复统计 gtest 外层/rostest 包装层，请按上述独立测试项理解。

## 默认桌面抓取

[acceptance2.json](validation/acceptance2.json) 使用 `scene.yaml` 中位于直接抓取路线的障碍箱。测试对第一次执行的起终点做独立直线插值，检测到 `obstacle_box / left_finger` 和 `right_finger` 碰撞；实际 MoveIt 路径完成绕障、接近、夹取、抬升。

| 指标 | 本次结果 |
|---|---:|
| 任务结果 | SUCCESS |
| Gazebo 实体目标高度增加 | 0.11993 m |
| 视觉平面定位误差 | 0.00104 m |
| 目标顶面深度高度误差 | 0.000027 m |
| 任务耗时（wall） | 35.07 s |

此记录包含碰撞膨胀、控制器插值检查、0.02 rad 关节限位余量，以及 ACM 同步后的最终配置。障碍箱 x=0.28 m；早期 `blocked_final.json` 使用 x=0.29 m，尚未加入限位余量。更早、障碍位于侧方的抓取回归保留在 `grasp_initial.json`、`grasp_final.json`，不能混为同一场景重复实验。

## 动态障碍与断流

早期基础场景的 [dynamic_obstacle.json](validation/dynamic_obstacle.json) 只向 Gazebo 加入新障碍，不调用 PlanningScene 添加物体。障碍经模拟 RGB-D → 点云 → OctoMap 导致执行取消，测得取消命令延迟约 0.813 s；随后有限次重规划，目标被持续阻挡时以 PLAN_FAIL 结束。不是穿过障碍完成任务。

[cloud_loss.json](validation/cloud_loss.json) 终止 obstacle_map，执行监控在约 1.901 s 后发出取消，报告 TIMEOUT。延迟为注入/断流到 Action cancel 的墙钟时间，不是物理机械臂的停止时间。

最终配置的恢复验收 **通过**，见 [recovery_verified.json](validation/recovery_verified.json)：

- 新障碍仅加入 Gazebo，经 RGB-D / OctoMap 检测，出现 `<octomap> / left_finger` 碰撞反馈并取消执行。
- 从注入到取消命令约 0.970 s；确认停止后等待自由空间观测更新，进行了两次恢复规划尝试。
- 共 4 次 trajectory execution，最终 SUCCESS，Gazebo 目标物体抬升 0.12004 m。
- 没有清空 OctoMap，没有通过 PlanningScene API 插入测试动态障碍。

早期失败记录 `recovery_before_limit_margin.json`、`recovery.json` 和 `recovery_acceptance.json` 分别保留限位、采样及地图更新时序问题。最终增加了已检查的 IK 目标、限位余量、ACM 同步和有界重试等待。OctoMap 对不可见区域会保守保留占据，障碍移走但不可见时仍允许任务安全失败，不无条件清图。

## OMPL

[ompl_tuned_5s.json](validation/ompl_tuned_5s.json) 使用旧观察位姿与额外中间障碍，起终点均有效、直线被阻挡；每条成功路径再次独立插值碰撞检查。每种规划器只有一次该配置试验。

| 规划器 | 结果 | wall 秒 | 关节空间路径长度 rad |
|---|---|---:|---:|
| RRTConnect | 成功且复核无碰撞 | 0.0712 | 0.981 |
| PRM | 成功且复核无碰撞 | 0.3510 | 5.487 |
| KPIECE | 成功且复核无碰撞 | 0.2116 | 1.024 |
| EST | 成功且复核无碰撞 | 0.0881 | 1.041 |
| RRTstar | 5 s 预算失败 | 5.0081 | — |

RRTstar 简单目标冒烟测试成功，见 [rrtstar_smoke.json](validation/rrtstar_smoke.json)；复杂阻挡场景 10 s 预算也有失败记录。五种配置可选不代表固定预算内必成功。运行时 `/grasp/metrics` 记录实际尝试的成功率、时间和路径长度。

## 范围与复现

按 [README](../README.md) 构建及启动；独立抓取、动态障碍、断流和恢复测试各自使用全新 sim.launch。最终源文件 SHA256 见 `validation/source_sha256.json`。原始过程日志含曾失败的开发迭代，用于保留问题证据；以本页标明的结果为准。

- 真机 launch / URDF 解析通过，记录 [real_parse.log](validation/raw/real_parse.log)；未做物理机械臂、夹爪或真实 D435i 验收。
- 仿真夹持采用近距离 fixed joint，验证实体抬升，不验证摩擦、夹紧力或六维力闭环。
- Top grasp 已做集成验收；Side grasp、真实 AprilTag 场景和实际相机标定待实物环境验证。YOLO 是检测器扩展接口。
- CHOMP/STOMP/TrajOpt 是明确返回不可用的扩展接口；当前实现时间参数化，不声称已运行这些优化器。
- 动态监控是地图更新、停止、有限重规划机制，不提供任意移动速度下的安全保证。真实停止距离、通信延迟和工具几何必须在后续联调验收。
