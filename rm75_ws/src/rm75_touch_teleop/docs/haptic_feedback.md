# RM75 → Geomagic Touch 触觉反馈

## 1. 定位与架构

本功能是原系统旁路增加的主端 **haptic rendering**，不是 RM75 阻抗控制器：

```text
Touch pose/orientation -> 原 rm75_touch_teleop -> RM75
RM75 Six_Force -> HapticRenderer -> internal command
                    -> geomagic_myhaptics/pubforce
                    -> /Geomagic/force_feedback -> device_node -> Touch
```

从端仍然是 Touch 控制 X/Y 与姿态；`CONTACT_MODE` 的 Z 仍只由原
`HybridForceControl` 导纳/力位外环控制。触觉数值从不写回 target pose、IK、
`JointPos` 或 HybridForceControl，因此不存在软件代数环，也没有新增机器人命令源。

`haptic_feedback_enabled` 默认 `false`，所有旧启动入口还会用 launch 参数再次覆盖为
false。功能关闭时 teleop 不发布内部设备命令，`pubforce` 也不会发送新增的零力消息，
所以旧 Touch 行为不变。

## 2. 实际设备接口与唯一 publisher

`geomagic_control/DeviceFeedback.msg` 的真实字段是：

```text
geometry_msgs/Vector3 force
geometry_msgs/Vector3 position
bool[] lock
```

`device_node.cpp` 将 `force.x/y/z` 交给 OpenHaptics `HD_CURRENT_FORCE`，单位按
OpenHaptics 接口和消息注释为 N。它只能输出三轴平移力，不能输出 `Mx/My/Mz`。
六维传感器的力矩仍只用于 RM75 原安全检查和诊断。

正常 `teleop.launch` 中，`geomagic_myhaptics/pubforce` 是
`/Geomagic/force_feedback` 的唯一 publisher。teleop 只发布私有内部 topic
`/rm75_touch_teleop/haptic_device_command`。不要同时启动旧的 `pubforce_single`、
`lockDoF.py` 或另一个 `pubforce`；它们也能声明同一个设备 topic。

设备 consumer 和 `pubforce` 均有 0.1 s 看门狗。内部命令断流后 publisher 持续发零；
publisher 异常退出后，device_node 自己也会在超时后通过 HD scheduler 将设备力清零。

## 3. 坐标与完整数学关系

位置映射与 Unitree Z1 的实际遥操作实现一致：

```text
v_robot = S v_touch
S = [ 0  0 -1
     -1  0  0
      0  1  0 ]
```

按虚功一致性，默认反向力轴映射为 `S^T`。传感器原始力先用原有明确的
`force_frame` 与 `force_frame_rotation_rpy` 转到 robot/base 坐标；这一步是物理
frame 变换。之后使用完全独立的 haptic rotation 和 sign：

```text
F_base = R(base <- force_frame) R_force_frame F_sensor
F_touch_raw = diag(haptic_force_sign) S^T R_haptic F_base
```

`force_sign` 仅供从端 Z 控制使用，**不进入 haptic 公式**；haptic 使用独立的
`haptic_force_sign` 和 `haptic_force_rotation_rpy`。即使当前默认全为单位变换，真机
也必须逐轴验证后再使用。

## 4. HapticRenderer 流水线

`HapticRenderer` 是无 ROS I/O 的纯 C++ 类：

1. 上述坐标映射与逐轴符号；
2. 一阶低通：`alpha=dt/(tau+dt)`，
   `F_filtered += alpha*(F_raw-F_filtered)`；
3. 连续 deadband：`sign(F)*max(|F|-deadband,0)`；
4. force scaling：乘 `haptic_force_scale`；
5. 虚拟阻尼：`F_damping=-B*V_touch`；
6. 逐轴 component clamp；
7. vector norm clamp；
8. vector-norm slew limit：`||delta F|| <= ramp_rate*dt`。

`/Geomagic/twist` 来自 device_node。源码先从 mm 位置估计速度，再除以 1000，故
teleop 明确按 m/s 使用；`B [N s/m] * V [m/s] = N`。阻尼各轴系数必须非负，测试
保证 `F_damping dot V <= 0`。

实际 `dt` 被限制到 `haptic_max_dt`；`dt<=0`、NaN、Inf 或非法向量立即清零。
普通 disengage、POSITION_MODE 和断流会按 ramp 回零；SAFE_STOP 与非法数值立即清零。

## 5. 输出门控与状态机交互

正式采样默认只有以下条件全部成立才允许环境力输出：

```text
haptic enabled
AND teleop active
AND CONTACT_MODE
AND force fresh
AND force_frame_verified
AND not SAFE_STOP
```

`POSITION_MODE`、两键都松开、灰键离合释放导致的力控复位、force timeout、
SAFE_STOP 和 shutdown 均回零。haptic 自身失效不会更改机器人 target、IK 或控制
状态；原六维力安全系统自己检测到超限时，仍会按原语义进入 SAFE_STOP。

Touch-only commissioning 配置是刻意的例外：`haptic_only_in_contact=false` 且
`haptic_require_teleop_active=false`，配合强制 `enable_motion=false`，使机器人完全
不运动时也能用人工小载荷逐轴核对 Touch 方向。

## 6. 参数

| 参数 | 正式/仿真初值 | 说明 |
|---|---:|---|
| `haptic_feedback_enabled` | false | 总开关 |
| `haptic_only_in_contact` | true | 只在 CONTACT 输出 |
| `haptic_require_teleop_active` | true | 要求任一离合有效 |
| `haptic_force_scale` | 0.02 | 从端力缩放 |
| `haptic_force_filter_tau` | 0.03 s | 独立 haptic 低通 |
| `haptic_force_deadband` | 0.5 N | 连续死区 |
| `haptic_damping_{x,y,z}` | 0 N s/m | 虚拟阻尼，现场从小值调 |
| `haptic_max_force_{x,y,z}` | 0.5 N | 逐轴软件限幅 |
| `haptic_max_force_norm` | 0.5 N | 合力限幅 |
| `haptic_force_ramp_rate` | 2.0 N/s | 合力变化率 |
| `haptic_force_timeout` | 0.1 s | 传感器 freshness |
| `haptic_max_dt` | 0.05 s | 算法 dt 上限 |
| `haptic_force_sign` | `[1,1,1]` | Touch 三轴独立符号 |
| `haptic_force_rotation_rpy` | `[0,0,0]` | 独立 haptic 旋转，rad |

0.5 N 不是对硬件持续/峰值能力的声明，只是保守软件 commissioning 值，
**REQUIRES HARDWARE VERIFICATION**。Touch-only 配置更低：scale 0.01、限幅 0.2 N、
ramp 0.5 N/s。不得直接把 RM75 的 50 N 原样送入设备。

## 7. ROS diagnostics

```text
/rm75_touch_teleop/haptic_enabled         std_msgs/Bool
/rm75_touch_teleop/haptic_raw_force       geometry_msgs/Vector3Stamped
/rm75_touch_teleop/haptic_filtered_force  geometry_msgs/Vector3Stamped
/rm75_touch_teleop/haptic_command         geometry_msgs/Vector3Stamped
/rm75_touch_teleop/haptic_force_fresh     std_msgs/Bool
```

前三个力 topic 均在 Touch/device 坐标系：raw 是完成 base→Touch 映射后的值，filtered
是独立低通后的值，command 是 deadband、scale、damping、限幅与 slew 后的实际目标。

## 8. 启动与关闭

旧入口行为不变：

```bash
./start_rm75_sampling_sim.sh
./start_rm75_sampling_real.sh          # haptic OFF, motion 仍默认 OFF
```

无 Touch 实物的仿真链检查：

```bash
./start_rm75_sampling_haptic_sim.sh gazebo_gui:=false \
  start_touch_driver:=false start_touch_helpers:=false
```

真机 Touch-only 低力验证（脚本强制 motion OFF）：

```bash
./start_rm75_sampling_haptic_real.sh
```

彻底关闭：停止 haptic 入口并使用任一旧入口；或显式
`haptic_feedback:=false`。不要在运行时仅杀掉 publisher 来代替正常关闭，尽管两级
watchdog 会兜底清零。

## 9. 方向验证（必须逐轴）

1. 确认 `enable_motion=false`，并检查 `/rm_driver/JointPos` 没有消息；
2. 启动 Touch-only 0.2 N commissioning；
3. `rostopic echo /rm_driver/UdpSixZeroForce`；
4. 沿已知 base +X、+Y、+Z 方向分别轻推采样器并记录 Fx/Fy/Fz；
5. 同时查看 raw、filtered、command diagnostics；
6. 确认实际 Touch 力是相应操纵方向的阻力；
7. 只调整 `haptic_force_sign`，确有轴旋转时才调整 haptic RPY；
8. 每次只验证一个轴，并重新启动节点使参数生效；
9. 三轴、回零、断流和节点退出均确认后，才能进入低力接触阶段。

不要以静止零漂的正负决定符号，也不要用“感觉差不多”跳过逐轴记录。

## 10. Commissioning 顺序

- Stage 0：`enable_motion=false, haptic_feedback=false`，只观察六维力与坐标。
- Stage 1：`enable_motion=false, haptic_feedback=true`，人工小载荷验证 Touch 三轴、
  deadband、ramp、限幅、断流和 shutdown 回零。
- Stage 2：仍使用 `rm75_touch_commissioning_real.yaml` 的低力从端参数。
- Stage 3：低速验证 Touch XY/姿态 + 原 Z 导纳 + haptic。
- Stage 4：现场安全确认后才考虑正式采样参数。

代码和脚本不会自动执行 Stage 2–4，也不会自动启用真实机器人运动。

## 11. 调参顺序

先保持 damping 为零，依次确认 mapping/sign、deadband、scale、component/norm limit、
ramp 和 watchdog；最后从很小的非负 damping 开始增加。haptic filter 与从端
`force_filter_tau` 完全独立，调整前者不会改变 RM75 Z 控制。
