# 单 AprilTag 五目标定位验收记录

日期：2026-10-17  
环境：ROS1 Noetic，Docker `rm75_noetic`，工作区 `/root/rm75_ws`

## 本轮实际完成

- 单公共 `apriltag16h5` ID=0 检测配置与调试图。
- 图像/CameraInfo 几何一致性、源时间戳 TF、相机新鲜度、PnP 正深度、重投影误差、像素尺寸、多帧平移/旋转稳定度质量门。
- `camera_link` 重复 TF 发布者诊断。
- 已确认的控制器 mm/rad + 固定轴 XYZ RPY 转换模块。
- 一次参考标定、五个 `marker→Link7` 和五个实时 `base→Link7` 的 SE(3) 计算。
- 标定 schema/版本/frame/手眼/规划末端完整性校验与原子 YAML 保存。
- 独立定位服务、五目标 JSON/PoseArray/选中目标话题。
- GUI 共用 marker 的 Tool1～Tool5 选择、识别、标定、预览、规划、执行和取消入口；图像角点叠加及五目标表格。
- J7 相机观察位捕获与自动运动、停稳后新鲜视觉门、MoveIt 目标/退回规划、24V 新鲜 ACK 取工具流程。
- 按用户要求不创建工具架或工具环境碰撞几何；保留 MoveIt 机器人自碰撞、关节限位和轨迹规划检查。
- 控制源互斥、关节新鲜度、标定版本、冻结视觉、物理验证与双授权执行门。
- 原多标记 `qr_task_real.launch`、旧 PICK/RETURN 状态机和测试保留。

## 实际执行结果

```text
catkin build rm75_qr_task rm75_sampling_gui --no-status
  All 15 dependency packages succeeded; 0 failed; 0 warnings.

roslaunch --files rm75_qr_task single_marker_five_targets.launch
  launch XML resolved successfully.

rm75_qr_task
  28 tests, 0 errors, 0 failures, 0 skipped

rm75_sampling_gui
  61 tests, 0 errors, 0 failures, 0 skipped

workspace aggregate
  245 tests, 0 errors, 0 failures, 0 skipped
```

新增加的 12 个五目标测试覆盖：

1. 原 marker 位姿回代五个示教 TCP。
2. marker 刚体平移。
3. marker 分别绕 X/Y/Z 旋转。
4. 接近 ±π 的姿态。
5. 非零 `Link7→TCP`。
6. Tool1～Tool5 独立选择且目标不重合。
7. 错误 marker ID。
8. 视觉超时。
9. 控制器定义/TCP 未确认与损坏/版本不匹配配置。
10. 原子 YAML 保存与加载。
11. `pose_reference: link7` 时不需要独立 `Link7→TCP`，且计算目标与 Link7 一致。
12. J7 观察位只改变 joint7，保留当前 joint1～joint6，并拒绝未验证观察位。

本轮未连接相机/机械臂做新的现场采样，未发送任何运动、电压或吸附命令。因此没有虚构重投影统计、重复定位精度、五个 `marker→Link7` 或到位误差。

## 当前明确状态

| 项目 | 状态 | 证据/原因 |
|---|---|---|
| 当前识别标记 | ID=0 / apriltag16h5 | 新默认检测配置 |
| PnP 边长 | 已确认 0.0225 m | 用户确认尺寸图中 22.50 mm 为编码黑方块；30 mm 外框不参与 PnP |
| 现场像素尺寸 | 约 11×11 px，拒绝 | 小于配置的 20 px 门，不能宣称毫米精度 |
| 手眼 | 配置中已确认 | 既有 `Link7→camera_link`，启动时检查变化/重复发布 |
| 控制器单位/旋转 | 已采用：mm/rad，固定轴 XYZ RPY | 驱动源码、只读 FK 审计和用户对官方示教器数据的确认 |
| 五点参考坐标 | `base_link→Link7` | 当前 MoveIt 末端为 Link7；用户确认五组为官方示教器末端位姿 |
| `Link7→TCP` | 不适用，不作为输入 | 本流程直接规划 Link7；代码不读取、不校验、不要求该变换 |
| `T_B_M_ref` | 未采集 | 不能唯一求五个 `T_M_Link7_i` |
| 五个 `T_M_Link7_i` | 当前不可用 | 等待采集一次公共 marker 参考位姿 |
| 五个当前视觉目标 | 当前不可用 | 等待合法参考标定且像素质量通过 |
| J7 相机观察位 | 等待首次标定捕获 | 与 `T_B_M_ref` 在同一次标定中原子保存 |
| 工具架/执行器环境几何 | 明确不使用 | 不创建虚拟 box；仍使用 MoveIt 自碰撞与关节限位检查 |
| MoveIt 真机执行 | 拒绝 | 多项物理门和授权为 false |
| 24V/吸附/取工具 | 已接入，默认未授权 | 只在到位误差合格后发送 24V，必须收到新鲜厂商 ACK |

## 真机只读与规划待验收

- 提高标签像素尺寸；建议优先改善观察距离/USB 3.x 分辨率，但不修改当前稳定相机链。
- 采集公共 marker 参考并生成实际标定 YAML；检查五点回代。
- 固定工具架，从不同观察姿态统计五个 Link7 目标的位置/姿态重复性、TF 延迟、遮挡和像素门。
- 建立工具架与五执行器碰撞几何，确认每个槽的 marker 坐标系接近方向和距离。
- 对五个“J7 观察位→最终点→观察位”逐个做 IK、机械限位、连续路径、自碰撞、速度/加速度和只规划验收。
- 在物理急停与监护人就位后，以 5% 速度逐个验证最终点和返回路径；记录实际 Link7 位姿、位置/姿态误差、停止结果和失败原因。

在上述证据完成前，真实执行被拒绝是预期且正确的结果。

现场验收完成后，上位机可在独立的监护验收授权下通过二次确认写入 `physical_validation.passed=true`；该操作本身不会运动或通电，也不会替代现场检查。
