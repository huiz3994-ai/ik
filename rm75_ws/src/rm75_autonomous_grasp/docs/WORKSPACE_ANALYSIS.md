# 工程分析与改造记录

## 原工程

工作空间为 `rm75_ws`，厂商代码位于 `src/rm_robot`，另有 Geomagic 遥操作、力位混合控制和示例包。实际 ROS1 Noetic 开发运行环境是 `rm75_noetic` 容器，宿主 ROS2 环境不能代替 ROS1 构建。

| 检查项 | 结果 |
|---|---|
| MoveIt 包 | 存在 `rm_75_moveit_config`、`rm_75_6f_moveit_config`，名称不是 `rm75_moveit_config` |
| robot_description | 来自 `rm_description` 的 URDF/xacro，由 launch 加载 |
| 模型 | RM75、RM75_6F、RM75_6FB 均有文件；控制器已自报 RM75-6FB，本系统固定选用 `Link7_6fb` |
| 关节 | `joint1` 至 `joint7`，限位以模型为准 |
| MoveGroup | `arm`；原末端 `Link7` |
| 控制器 | 仿真 ros_control FollowJointTrajectory；厂商真机 `rm_control` 对接 `rm_driver` |
| 夹爪 | 厂商 Gripper_Set / Gripper_Pick 命令及结果 ACK，尚无已确认的实际夹爪/夹持检测 |
| TF | 原模型提供 base_link 到 Link7；新系统必须加入 world、手眼安装、相机 optical、TCP |

原工程具备机器人网格、关节模型、MoveIt/OMPL 配置、Gazebo 控制和厂商驱动；但不具备完整 RGB-D 目标定位、眼在手上标定配置、动态地图监控、抓取状态机和自动验收闭环。原标准型号的 planning_execution 入口使用 `/arm/joint_states`，不能直接当作完整真机启动入口。原 rm_control 的取消/完成判断不足以承担本任务要求的执行监控。

## 新增方案与文件所有权

不修改厂商模型/旧遥操作逻辑，新增两个 ROS 包：

- `rm75_vision_grasp`：ObjectDetector、颜色/AprilTag、DepthToPose、RGB-D 同步与 tf2，以及感知测试。
- `rm75_autonomous_grasp`：grasp、planning、state_machine、execution、gripper、action、urdf、worlds、launch、config、scripts、test 和 docs。

`system.urdf.xacro` 包装厂商六维力模型，添加眼在手上测试安装和夹爪；`system.srdf` 将规划链延伸到 grasp_tcp。`sim.launch` 和 `real.launch` 共享感知、地图、规划和任务逻辑，分别接仿真控制器与带反馈/停止确认的真机轨迹桥接。应用始终使用 MoveIt trajectory execution。

完整文件入口和启动命令见 [README](../README.md)，模块契约见 [架构](ARCHITECTURE.md)，实际完成的测试与未验证范围见 [验收记录](VALIDATION.md)。真实夹爪、TCP、手眼参数和现场停止距离待硬件到位后按 [真机联调](REAL_COMMISSIONING.md) 填写。
