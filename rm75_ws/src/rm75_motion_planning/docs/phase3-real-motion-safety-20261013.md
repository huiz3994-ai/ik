# RM75-6FB 第三阶段：Link7 真实运动验证与在线安全执行层

更新日期：2026-10-14

## 1. 边界与真实证据

本阶段固定使用 `base_link -> Link7`，不引入工具模型、TCP 标定或 Tool
Frame 切换，也不修改 Touch 映射、力控和 `rm_driver`。后续安装采样工具、
夹爪或力传感器后，再独立增加 TCP 层。

当前唯一已经完成的真机运动证据仍是第二阶段 joint7 `+0.02 rad` 往返：
135 点、50 Hz、2.68 s；最大实际位移 0.017450 rad，最大七轴跟踪误差
0.010097 rad，返回误差 0.004733 rad，峰值合力 1.381063 N，峰值力矩
0.071986 N·m；碰撞 0/400，未急停且无驱动错误。该数据不能外推为
Cartesian、多轴、奇异区或 5×100 次验收结果。

本次完成的是第三阶段软件入口、Guard、采集和测试。没有在本次会话中
启动 `rm_driver` 或执行新真机动作，因此 X/Y/Z、Rx/Ry/Rz、组合 6D 和
真实 benchmark 的结果仍为“待监督实测”，没有使用 Gazebo 数据代替。

## 2. Link7 Cartesian 执行链

`rm75_cartesian_commissioning` 实现：

```text
当前 JointState -> FK(base_link -> Link7) -> Link7 目标 Pose
  -> SE(3) 轨迹 -> 连续 IK -> 20 ms jerk 限制轨迹
  -> 离线 FK/动力学/FCL 预检
  -> CommandSourceManager + RuntimeGuard
  -> /rm_driver/JointPos
  -> JointState -> FK -> Link7 误差统计
```

参数硬边界：每个平移轴不超过 10 mm，每个姿态轴不超过 5°，Link7
线速度 0.01–0.03 m/s，角速度不超过 10°/s，关节速度/加速度/jerk
默认不超过 0.05/0.10/0.50 SI。低速要求现在通过降低动力学上限并重新以
20 ms 采样生成轨迹实现，不再只缩放时间戳。

目标与实际均输出 XYZ 和 quaternion；实际值严格按要求由
`JointState -> FK -> base_T_Link7` 得到。输出还包括最终、最大和 RMS
位置/姿态误差、最大关节变化、IK 调用/细分/fallback 状态、控制频率、
指令周期、反馈 age、丢帧估计、速度、加速度、力和力矩峰值。

入口默认 `execute=false`，此分支不会创建 `/rm_driver/JointPos` 发布者：

```bash
# 仅规划/预检 X +10 mm
roslaunch rm75_motion_planning cartesian_commissioning.launch dx:=0.01

# 只有完成现场检查且由操作者明确授权后，单次执行
roslaunch rm75_motion_planning cartesian_commissioning.launch \
  dx:=0.01 execute:=true
```

姿态参数为弧度，例如 Rx +5° 使用 `droll:=0.08726646`。组合 6D 可同时
给出 XYZ/RPY，但首次现场验收必须按 X、Y、Z、Rx±、Ry±、Rz± 的顺序
逐项执行和返回，不允许无人值守批量连续运行。

## 3. Runtime Guard

每个 20 ms 周期输入当前状态、上一指令、下一目标、下一指令速度和反馈
时间戳。检查顺序为：

1. 维度和有限值；
2. JointState feedback timeout；
3. 当前状态、下一目标以及 `q_current + dq_command * 0.02` 的关节余量；
4. 当前状态相对上一指令的最大单关节跟踪误差；
5. 当前状态与下一目标的 MoveIt/FCL 碰撞；
6. 外层执行器继续检查力/力矩超限、力反馈超时、`Plan_State=false` 和
   `/rm_driver/Error` 非零错误码。

`RuntimeGuardResult` 同时提供 `allow`、`collision`、`timeout`、
`tracking_error`、`joint_limit`、`max_joint_error`、`reason` 以及
`ALLOW/WARN/STOP`。默认 STOP
阈值为反馈 age 0.10 s、跟踪误差 0.03 rad、关节余量 0.02 rad；WARN
从跟踪阈值 80% 或关节余量 0.05 rad 开始。缺少碰撞检查器时 fail closed。

STOP 时停止流式指令，发布现有 `/rm_driver/Emergency_Stop` 请求，并观察
两秒，记录停止后的最大单关节位移。真实 E-stop ACK 最坏延迟和制动距离
尚未测量，不能从代码测试推断。

## 4. 统一命令入口

```text
Touch ------+
MoveIt -----+-> CommandSourceManager -> RuntimeGuard -> controller adapter
Autonomous -+
```

`GuardedCommandGateway` 把 execute 授权、唯一命令源所有权和 Runtime
Guard 合并成一次判定。低速 joint7、Cartesian 和 Gazebo shadow 入口已经
使用该接口。三个入口还会在 advertise 前查询 ROS master；命令 topic 已有
任意 publisher 或 master 查询失败时均拒绝执行。现有 Touch 源码和映射按
本阶段禁令没有修改；在 Touch adapter 迁移到该 gateway 之前，旧 Touch
节点仍不得与第三阶段执行器同时启动。
未来接入必须通过 gateway，不能增加旁路发布路径。

## 5. 真实执行 Benchmark

`benchmark/real_execution/` 定义五个场景：`joint_small`、
`cartesian_line`、`cartesian_orientation`、`random_small_6d` 和
`near_singular`。每个场景记录 100 次“尝试”，STOP/FAIL 也计入 100 次，
不能删除失败后补跑到 100 个 PASS。

`record_real_trial.py` 只把一个已经结束的监督执行日志追加到 CSV，本身不
启动 ROS、不运动机械臂；它只接受同时包含 `data_source=real_robot` 和
`reference_frame=base_link_to_Link7` 的日志。`analyze_real_benchmark.py`
输出汇总 CSV、JSON、Markdown 和 SVG plot；任一场景少于 100 次时报告为
`INCOMPLETE` 并返回 3。

当前模板没有真实第三阶段数据，因而没有生成或提交虚构的数值报告。

## 6. Gazebo Shadow

`rm75_gazebo_shadow_execution` 保留为独立仿真通道：强制
`/use_sim_time=true`，feedback/command 只能位于 `/arm/...`，拒绝任何
包含 `rm_driver` 的 topic，默认 `execute=false`。它复用轨迹参数化、FCL、
Command Manager 和 Runtime Guard，用于大范围、极限姿态、障碍物和故障
注入，不写入 `benchmark/real_execution`。

已有 shadow 证据：joint7 `+0.05 rad` 往返 117 点、2.32 s，最大关节跟踪
误差 0.00289347 rad，Guard WARN 0，反馈 49.99843415 Hz；包围盒负例在
创建控制器 publisher 前被 FCL 预检拒绝。这些只属于仿真。

## 7. 测试覆盖与限制

GTest 覆盖连续 IK/FK、Guard 正常/timeout/跟踪超限/关节超限/速度预测/
碰撞、STOP 距离、execute 默认关闭/显式打开、唯一命令源与 guarded
gateway。Python 自动测试覆盖 5×100 报告生成和不满 100 次时的
`INCOMPLETE` 结果。

验证结果：`catkin build rm75_motion_planning --no-deps` 无警告成功；27 个
GTest 全部通过；Python benchmark suite 4 个测试通过；带 ROS 环境运行
CTest 为 2/2 PASS。`catkin_test_results` 显示 54 tests、0 errors、
0 failures（该工作区会把 GTest suite/testcase 聚合计数两次）。

仍待现场完成：X/Y/Z +10 mm 及返回、Rx/Ry/Rz ±5°、一条小范围组合
6D、真实停止距离、完整工作站障碍物、五场景各 100 次。Link7 的
JointState-FK 误差是闭环模型指标，不是外部测量精度；绝对末端精度仍需
外部量具或视觉测量。
