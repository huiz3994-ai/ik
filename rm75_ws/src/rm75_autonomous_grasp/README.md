# RM75 自动视觉抓取与动态避障（ROS1 Noetic）

包括独立的 `rm75_vision_grasp` 感知包和 `rm75_autonomous_grasp` 任务包。复用已由控制器确认的 RM75_6FB（`Link7_6fb`）模型，眼在手上 RGB-D，MoveIt + OMPL + FCL + OctoMap，仿真和真机入口分离。真机尚未实测，实际夹爪和手眼参数待配置，默认禁止执行。

## 构建

本工程 Noetic 环境在 `rm75_noetic` 容器中：

```bash
docker exec -it rm75_noetic bash
source /opt/ros/noetic/setup.bash
cd /root/rm75_ws
catkin build rm75_vision_grasp rm75_autonomous_grasp -j 2
source devel/setup.bash
```

依赖声明见 package.xml。现有容器已具备 MoveIt、OMPL、FCL、PCL、OpenCV、Gazebo 11、ros_control；无图形会话时需 `xvfb`。真机相机另需 ROS1 `realsense2_camera`。AprilTag 测试采用 OpenCV ArUco 的 AprilTag 36h11 字典，不依赖 apriltag_ros；YOLO 通过 ObjectDetector 接口扩展。

## 仿真

停止旧 RM75 仿真/遥操作，确保同一 master 只有一个运动入口。使用有图形环境的终端：

```bash
export GAZEBO_PLUGIN_PATH=/root/rm75_ws/devel/lib:${GAZEBO_PLUGIN_PATH:-}
roslaunch rm75_autonomous_grasp sim.launch gui:=true rviz:=true
```

无界面测试也需要深度相机渲染环境：

```bash
export GAZEBO_PLUGIN_PATH=/root/rm75_ws/devel/lib:${GAZEBO_PLUGIN_PATH:-}
export LIBGL_ALWAYS_SOFTWARE=1
xvfb-run -a -s '-screen 0 1024x768x24' \
  roslaunch rm75_autonomous_grasp sim.launch gui:=false
```

启动会通过 MoveIt 到达观察位姿，不直接发布 JointState 控制。等待 `/observation_ready` 为 true 后在另一个已 source 的终端运行：

```bash
rosrun rm75_autonomous_grasp preflight.py
rosrun rm75_autonomous_grasp run_grasp.py _mode:=top
```

测试成功后机器人保持夹持，不自动松手。每次独立回归测试请完整关闭并重启 sim.launch，避免上一轮附着物和地图污染。`_mode:=side` 提供侧抓候选；默认桌面测试针对 top，侧抓需要对应的可达场景和工具姿态验收。

## 自动验收

```bash
# 无 Gazebo 的单元测试
catkin test rm75_vision_grasp rm75_autonomous_grasp --no-status
catkin_test_results build/rm75_vision_grasp/test_results
catkin_test_results build/rm75_autonomous_grasp/test_results

# 每项使用新启动的仿真
rosrun rm75_autonomous_grasp integration_test.py
rosrun rm75_autonomous_grasp dynamic_test.py _mode:=obstacle
rosrun rm75_autonomous_grasp dynamic_test.py _mode:=cloud_loss
# 可选恢复测试：取消后移走障碍，等待地图更新并重规划
rosrun rm75_autonomous_grasp dynamic_test.py _mode:=recovery
```

完整抓取测试检查真实 Gazebo 目标高度增加 >8 cm。dynamic_test 的障碍只加入 Gazebo，必须经相机点云/OctoMap 进入碰撞世界；它放在最终目标上方阻塞接近，预期停止并报告失败，而不是穿过障碍。cloud_loss 测试终止本系统 obstacle_map 节点，预期超时停止。测试均输出 JSON，具体实测结果见 `docs/validation` 和 `docs/VALIDATION.md`。

规划器比较（只规划、不执行；构造起终点有效但直线被挡住的路径）：

```bash
rosrun rm75_autonomous_grasp planner_benchmark.py _planning_time:=5.0
```

结果包括失败和超时，不承诺各随机采样规划器在固定预算内全部成功。

还可手动注入同时出现在物理世界和 PlanningScene 的确定性障碍：

```bash
rosservice call /scene/dynamic_obstacle 'data: true'
rosservice call /scene/dynamic_obstacle 'data: false'
```

## RViz 验证

- Fixed Frame 使用 `world`，MotionPlanning 的 Planning Group 选择 `arm`，末端 `grasp_tcp`。
- 显示 RobotModel、TF、PlanningScene；查看 `/move_group/monitored_planning_scene` 中的场景和 OctoMap。
- 添加 PointCloud2 `/mapping/cloud`，观察相机移动时点云是否固定在正确世界位置。
- 添加 Pose `/perception/object_pose` 与 Image `/camera/color/image_raw`，对照红色目标。Pose 的检测方向不是物体姿态；抓取方向由 GraspPlanner 产生。
- 检查桌面、箱体、圆柱与 Gazebo 一致，规划线不能穿过障碍。移动障碍后查看 `/grasp/state`、Action result、控制器 cancel。
- 抓取后检查目标从 CollisionObject 转为 AttachedCollisionObject，抬升时随 TCP 运动且仍参与碰撞检查。

## 配置

`config/task.yaml`：规划器 RRTConnect/RRTstar/PRM/KPIECE/EST、速度/加速度比例、重规划次数、地图超时、抓取偏移及真机启用条件。通过 `task_config:=/绝对路径/配置.yaml` 加载自定义配置。

`config/ompl.yaml`：显式配置五种规划器和碰撞采样；`config/scene.yaml`：Gazebo 与 PlanningScene 共用几何；`config/sensors.yaml`：OctoMap updater；`config/sim_control.yaml`：仿真控制器容差；`urdf/system.urdf.xacro`：眼在手上模型和测试夹爪。

CHOMP/STOMP/TrajOpt 为明确报错的扩展接口，当前启用时间参数化。没有实装的优化器不能宣称已经运行。

- [原工程分析与改造](docs/WORKSPACE_ANALYSIS.md)
- [系统架构与接口](docs/ARCHITECTURE.md)
- [真机准备与联调](docs/REAL_COMMISSIONING.md)
- [故障排查](docs/TROUBLESHOOTING.md)
- [验证记录](docs/VALIDATION.md)
