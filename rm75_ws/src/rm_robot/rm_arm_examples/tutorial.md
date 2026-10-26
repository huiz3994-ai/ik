# RM Arm Examples 源码课程大纲

## 项目用途与学习目标

本目录包含 3 个 ROS1 真机示例，用来学习通过 `rm_driver` 的话题接口读取 RM 机械臂状态、发送基础运动指令，以及执行六维力传感器参与的力位混合运动。本课程的目标是先安全读状态，再理解普通运动，最后理解高风险的力控流程。

## 源码版本与本地状态

- 检查日期：2026-09-30。
- 工作区：`/root/rm75_ws`（宿主机映射为 `/home/zjh/project/RealMan/rm75_ws`）。
- 当前目录不是可识别的 Git 仓库，因此无法给出提交号。
- 三个包均已被当前 catkin 工作区识别。
- 本地与结论相关的已知改动位于 Geomagic Touch 驱动，不影响本目录三个 demo 的接口分析。

## 前置基础

- ROS1 的节点、Publisher、Subscriber、topic 和 launch 文件。
- RM75 真机工作空间、TCP/UDP 网络配置与急停操作。
- 关节空间运动与笛卡尔空间运动的区别。
- 运行力控示例前，需要理解六维力传感器坐标系、力方向和安全限值。

## 覆盖范围

本大纲覆盖：

- `get_arm_state`：查询版本、关节/末端状态和六维力数据。
- `control_arm_move`：依次演示 MoveJ、MoveJ_P、MoveL、MoveC。
- `force_position_control`：开启力位混合控制、执行 MoveL、关闭力控。

暂未覆盖：`rm_driver` 内部网络协议实现、其他型号机械臂示例、MoveIt/Gazebo、夹爪/灵巧手、Modbus，以及完整的六维力标定流程。

## 课程 01：怎样只读地查询机械臂状态？（大纲）

读完能回答：demo 如何发出一次查询请求，并通过回调打印控制器版本、关节、末端位姿和六维力？

主链：启动 `rm_driver` → 启动 demo → 发布版本/状态/力查询 → `rm_driver` 返回结果话题 → 回调打印数据。

必读路径：

1. `get_arm_state/launch/get_arm_state_demo.launch`：无输入参数；启动 `api_Get_Arm_State_demo`。
2. `get_arm_state/src/api_Get_Arm_State_demo.cpp::main`：创建 3 个查询 Publisher 和 7 个结果 Subscriber；输出是终端日志。
3. `Get_Arm_Software_version_Callback`：输入版本结果；输出控制器各模块版本。
4. `GetArmState_Callback` 与 `Get_Arm_State_Callback`：输入两种状态消息；输出角度/弧度、欧拉角/四元数。
5. 四个 `*_Force_Callback`：输入不同参考系下的六维力；输出 Fx/Fy/Fz/Mx/My/Mz。

选读证据：`get_arm_state/README.md` 的示例输出。

暂缓阅读：运动命令与力控，由课程 02、03 负责。

## 课程 02：四种基础运动命令怎样串行执行？（大纲）

读完能回答：MoveJ、MoveJ_P、MoveL、MoveC 分别控制什么，demo 如何借助 `/rm_driver/Plan_State` 串起它们？

主链：选择 RM75 launch → 设置 `Arm_Dof=7` → 发布 MoveJ → 收到完成状态 → 发布 MoveJ_P → 发布 MoveL → 发布 MoveC → 等待结果。

必读路径：

1. `control_arm_move/launch/rm_75_move_demo.launch`：把 7 自由度参数传给节点。
2. `control_arm_move/src/api_move_demo.cpp::main`：构造预置目标点并发布到 4 个命令话题。
3. `planStateCallback`：输入 `/rm_driver/Plan_State`；更新 `arrival_flag`，驱动下一阶段。
4. MoveJ 数据段：输入 7 个关节目标；输出关节空间运动。
5. MoveJ_P、MoveL、MoveC 数据段：输入笛卡尔位姿；输出关节规划到位、直线轨迹和圆弧轨迹。

选读证据：`control_arm_move/README.md`；`rm_msgs` 中四种消息定义。

重要分支：`Arm_Dof` 为 6 或 7 时使用不同关节数组。

已发现风险：MoveL 发布后的等待条件写成 `arrival_flag == 3`，而发布时通常仍为 2；这会形成竞态，可能在 MoveL 尚未完成时继续判断 MoveC。未经修复和空载验证，不建议直接在真机运行。

暂缓阅读：力位混合参数，由课程 03 负责。

## 课程 03：力位混合控制如何开启、运动和关闭？（大纲）

读完能回答：demo 怎样让末端在笛卡尔运动期间沿指定轴保持目标力，以及为什么必须保证关闭命令执行？

主链：MoveJ_P 到初始位姿 → 设置六维力模式 → 等待设置成功 → MoveL → 等待到达 → 停止力位混合 → 等待停止成功。

必读路径：

1. `force_position_control/launch/force_position_control_demo.launch`：启动 `api_Force_Position_Control_demo`。
2. `api_force_position_control_demo.cpp::main` 的 MoveJ_P 段：输入预置笛卡尔位姿；输出安全起始姿态。
3. `Set_Force_Position` 构造段：`sensor=1`、`mode=0`、`direction=2`、`N=5`；输出工作坐标系 Z 轴 5 N 的力控请求。
4. MoveL 段：输入目标位姿；输出力控状态下的直线运动。
5. `StopForcePosition` 段和三个回调：根据执行结果推进并退出力控。

选读证据：`force_position_control/README.md`；`rm_msgs/Set_Force_Position` 消息定义。

重要分支：设置、运动或停止失败时，当前示例仅打印错误或持续等待，没有超时和自动急停兜底。

暂缓阅读：传感器标定、碰撞阈值、控制器内部力环，当前目录没有实现证据。

## 推荐实操顺序

1. 只运行课程 01，确认通信、型号、关节数和状态数据正确。
2. 阅读并修复课程 02 的等待竞态，核对全部目标点，在低速、空载、有人持急停的条件下测试。
3. 完成六维力标定和安全评审后，再测试课程 03；首次测试应降低目标力和运动范围，并准备独立停止手段。
