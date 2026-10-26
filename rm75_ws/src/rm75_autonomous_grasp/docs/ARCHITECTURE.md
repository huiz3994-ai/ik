# RM75 RGB-D 抓取与动态避障架构

## 适用范围和配置所有权

ROS1 Noetic / MoveIt 1；用户硬件为控制器已确认的 RM75-6FB，带六维力传感器、D435i 安装末端。本实现固定复用仓库 `RM75_6FB` 的 `Link7_6fb` 模型；工具 TCP 和相机外参仍必须按实物测量。原有遥操作和厂商包不修改，新入口独立选择真实/仿真 controller YAML。**遥操作、旧 rm_control 和新 trajectory_bridge 不得同时拥有控制权。**

夹爪型号、工具几何、手眼标定均未提供。system.urdf.xacro 的夹爪和相机安装参数仅供仿真。真机 execute 默认 false，桥接层和任务层分别检查启用条件，防止从 RViz 绕过任务层的未配置检查。

## 数据与控制接口

| 生产者 → 消费者 | 接口 | 契约 |
|---|---|---|
| D435i / Gazebo → VisionNode | `/camera/color/image_raw`、`/camera/aligned_depth_to_color/image_raw`、`/camera/color/camera_info` | 近似同步 ≤50 ms；RGB/深度已对齐；米或毫米编码明确 |
| ObjectDetector → DepthToPose | Detection{name,bbox,pixel,score} | 默认红色，AprilTag 36h11，YOLO 实现相同虚接口 |
| DepthToPose → tf2 | optical PoseStamped | 5×5 有效深度中位数，IQR 离群检查；无效深度拒绝 |
| VisionNode → 任务 | `/perception/object_pose` | base_link；使用图像时间戳；位置估计不是物体六维姿态估计 |
| VisionNode → 调试 | `/perception/object_pose_camera`、`/perception/detection` | camera_link 的 PoseStamped；物体名/bbox/pixel |
| 相机 → ObstacleMap | `/camera/depth/color/points` | 保留观测时间戳 |
| ObstacleMap → MoveIt | `/mapping/cloud` | base_link 下 ROI/体素处理后变回相机坐标，保留正确的射线原点 |
| MoveIt → 执行监控 | `/move_group/monitored_planning_scene` | 包含 OctoMap、CollisionObject、ACM、AttachedCollisionObject |
| 用户 → 任务 | `/grasp` Action | `grasp_mode=top/side`，反馈状态，结果 success/error，支持取消 |
| 任务 → MoveIt | planning + `/execute_trajectory` Action | 应用不直接发送 JointState/关节命令 |
| MoveIt → 仿真 | `/arm/arm_joint_controller/follow_joint_trajectory` | ros_control |
| MoveIt → 真机桥接 | `/rm_group/follow_joint_trajectory` | 实际关节反馈、时间插值、容差、超时、停止确认 |
| 真机桥接 → 厂商驱动 | `/rm_driver/JointPos` | 厂商消息；只由底层桥接发布 |
| Gripper → 厂商驱动 | Gripper_Set / Gripper_Pick | 必须增加实物适配的 `/gripper/object_detected` Bool；命令 ACK 不等于夹住物体 |
| 任务 → 指标 | `/grasp/metrics` | JSON：规划器、成功率、耗时、关节空间路径长度 rad |

TF：`world → dummy → base_link → … → Link7 → camera_link → optical frames`；`Link7 → grasp_tcp`。仿真 URDF 提供 optical TF，真机 RealSense 驱动提供设备内部 TF，手眼安装 TF 由 URDF 提供，禁止重复发布同一 child frame。

## 状态与恢复

`IDLE → WAIT_OBJECT → VISION_DETECT → POSE_ESTIMATION → UPDATE_OBSTACLE_MAP → PLAN_PATH → EXECUTE_TRAJECTORY → GRASP → LIFT → SUCCESS`。

预抓取、接近、抬升分别规划和执行。候选包括 top/side 和四个 yaw，使用 TCP 偏移、IK、关节限位、FCL 全路径筛选。故障状态包括 VISION_FAIL、TF_FAIL、IK_FAIL、COLLISION、PLAN_FAIL、TIMEOUT、CANCELLED。错误字符串保留底层原因。发生夹取后失败时保留附着物，不自动松手。

动态监控在执行期间以 10 Hz 目标频率检查剩余路径，实际频率取决于 CPU 和碰撞几何。检测危险时取消 MoveIt Action，发送停止请求（真机），读取新鲜关节反馈确认停止，再做有限次重规划。每次重试前用 `replan_map_wait`（默认 1 s）等待自由空间观测更新；恢复期间的暂时 IK 不可行不会立即耗尽整个任务，重试次数仍有上限。场景更改但未影响剩余路径时继续运行。关节或点云过期停止执行。**这不是安全认证控制器，也不是对任意速度移动障碍的时间参数化预测规划。**

FCL 检查包含关节空间线性段，以及控制器使用的三次/五次插值（轨迹带速度/加速度时），采样间隔不超过 20 ms，同时使用 `collision_joint_step` 限制关节采样步长；默认向 PlanningScene 写入 5 mm 机器人碰撞膨胀。规划另加 0.02 rad 关节限位余量，避免目标落在机械止挡附近；测量状态超限仍然停止。必须按障碍尺寸、运动速度和测量/停止延迟验收。对很薄障碍应减小分辨率。不要把离散检查描述成连续碰撞检测。

## 障碍、目标与附着物

Gazebo 的桌面/箱体/圆柱与 MoveIt 碰撞几何来自同一 scene.yaml。Gazebo 的目标真值只用于生成和测试，感知不读取真值。目标通过颜色/深度定位后进入 PlanningScene；MoveIt 的世界物体形状过滤器排除已建模的目标，避免目标重复出现在 OctoMap 和 CollisionObject；PCL 的额外目标 ROI 排除默认关闭，不能因检测到红色就抹掉未知障碍。接近阶段只允许夹爪链接与目标接触，不豁免机械臂、相机或桌面。

OctoMap 使用自由空间射线更新，不做无条件周期全图清空。移走但不可见的障碍可能保守残留；需要重新观察相应区域。新图/场景未就绪时不得开始运动。地图新鲜度同时检查 `/mapping/processed` 与 MoveIt updater 的 `/move_group/d435i/self_filtered`；静态 OctoMap 没有几何变化时不误判断流。部署时必须标定点云噪声、地图分辨率和安全距离。

## 仿真夹爪边界

测试夹爪含几何和碰撞模型；`rm75_sim_gripper` 在 TCP 与目标中心距离小于 35 mm 时建立 Gazebo fixed joint。抬升验证读取 Gazebo 目标高度，证明物体在物理世界跟随运动。**这是理想化抓取约束，不验证真实指爪摩擦、夹紧力、滑落或六维力闭环。** 后续实际夹爪确定后替换模型/控制适配，不改变任务层接口。

## 优化接口

OMPL 规划后经过 TrajectoryOptimizer、时间参数化及 FCL 再检查。当前实现 `none`/`time_parameterization`（均生成可执行时间戳）。CHOMP 已安装但尚未适配为此系统的后处理器；STOMP、TrajOpt 预留接口。选择未实现的优化器明确失败 `OPTIMIZER_UNAVAILABLE`，不伪装成优化成功。接入优化器必须保留起终点/关节限制，输出 RobotTrajectory 并通过同一验证链。

## 指标定义

成功率分母为实际发出的规划尝试，不含前置 IK/输入拒绝。路径长度为所有关节增量的 L2 范数累加，单位 rad，不等于 TCP 米制距离。耗时使用 wall clock。测试报告应记录场景、随机种子或重复次数、规划器参数和机器环境；单次成功不能作为生产成功率承诺。
