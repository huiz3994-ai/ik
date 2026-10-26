# 故障排查

| 现象 | 检查与处理 |
|---|---|
| 找不到包/错误 ROS 版本 | 进入 rm75_noetic 容器，source Noetic 和 /root/rm75_ws/devel/setup.bash；不要在宿主 Jazzy 下构建 |
| 图像 Topic 有订阅者但无发布者 | 检查 Gazebo server 日志是否 Can't open display / Rendering disabled；使用 xvfb-run 启动。gui:=false 不表示深度相机不需要 OpenGL |
| TF_FAIL | 检查 world、base_link、camera_link、optical frame、grasp_tcp；确认一个 child 只由一个发布者负责；检查消息时间戳和 use_sim_time |
| 目标偏移/轴反向 | 深度反投影属于 optical frame；检查外参与深度单位，不能把 optical frame 名称直接改成 camera_link |
| VISION_FAIL | 红色目标是否可见，图像/深度是否对齐，CameraInfo 尺寸和内参是否匹配；检查 5×5 深度和最小深度 |
| MAP_EMPTY / stale cloud | 检查 /mapping/cloud、/mapping/processed、/move_group/d435i/self_filtered、/move_group/monitored_planning_scene；确认 OctoMap updater 加载和 TF 可用 |
| 移走障碍仍存在 | OctoMap 未观测到自由空间，重新观察遮挡区域；不要在机械臂运动中清空整张图 |
| IK_FAIL | 检查末端模型和 TCP、目标是否可达、姿态约束；KDL 搜索可能受超时限制，可增加 IK 超时或配置已安装的 TRAC-IK 后再次验收 |
| JOINT_LIMIT / joint limit margin | 实测关节超限会停止；规划默认留 0.02 rad 余量。检查工具可达性和实测跟踪误差，不能扩大模型限位掩盖问题 |
| 起点碰撞 | 在 RViz 看机器人和工具几何/OctoMap；检查关节初值、工具 ACM。不能通过全局允许碰撞绕过 |
| 仿真 TF 与渲染位置偏差 | 检查底座是否穿入地面；本 world 将地面降至 -0.05 m，避免固定底座与地面接触冲突。不要使用伪造 TF 修正物理模型错误 |
| RRTstar 超时 | 该优化规划器在七轴狭窄绕障场景可能在预算内只得到近似解；近似解被拒绝。增加预算、调整参数或选择已验证的 RRTConnect，不将近似路径用于执行 |
| 控制器等待 | 仿真应连接 /arm/arm_joint_controller/follow_joint_trajectory；真机应连接 /rm_group/follow_joint_trajectory；不能使用旧公共入口中混用的配置 |
| STOP_UNCONFIRMED | 不再自动重规划；检查新鲜关节、底层停止 ACK、实际设备状态 |
| GRIPPER_NOT_CONFIGURED | 配置真实夹爪反馈适配器和已验证标志 |
| OPTIMIZER_UNAVAILABLE | CHOMP/STOMP/TrajOpt 后处理尚未接入；当前选 time_parameterization，不能把占位接口当成功实现 |
| OBJECT_ALREADY_HELD | 上次成功后物体仍附着；明确释放并清理 PlanningScene 后再开始任务。仿真回归建议重启场景 |
| Gazebo No p gain specified | 厂商 position interface 未配置 PID 时回退直接位置接口；确认控制器实际 tracking，不能仅根据该日志判定执行失败 |

日志：`/grasp/state`、`/grasp/metrics`、ROS Action feedback/result、`~/.ros/log`、`~/.gazebo/server-*/default.log`。点云发布频率、规划时间和停止延迟应同时记录。
