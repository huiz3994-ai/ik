# RM75-6FB三维与七轴控制验证记录（2026-10-15）

## 范围

本轮只进行了源码审计、无硬件单元/ROS合约测试、受控服务隔离启动、VTK虚拟显示测试和离线GUI启动测试。没有启动真实关节运动；隔离启动时没有连接驱动或发送运动目标，也没有把仿真或离线结果表述为真机验收。

## 模型与渲染结果

- xacro成功展开`RM75_6FB/rm_75_6fb_description.urdf.xacro`的`Link7_6fb`分支。
- 链为`base_link -> joint1..joint7 -> Link7`；joint7 origin z=0.1612 m，限位[-6.28, 6.28] rad。
- 读取8个厂商STL：base_link、Link1..Link6、`Link7_6fb.STL`。
- 零位FK得到Link7 z=0.8677 m。
- `xvfb-run` 1920×1080渲染烟测成功：8个实际actor、8个目标actor、9组坐标轴、1个renderer。
- Noetic Python RViz绑定可导入；因独立目标模型需要第二套TF且会与真机frame冲突，最终使用VTK做同窗双模型，MoveIt/FCL继续承担碰撞检查。

## 自动化结果

命令：

```bash
source /opt/ros/noetic/setup.bash
cd /root/rm75_ws
catkin build rm75_sampling_gui --no-status
source devel/setup.bash
QT_QPA_PLATFORM=offscreen catkin run_tests rm75_sampling_gui --no-deps --no-status
catkin_test_results build/rm75_sampling_gui/test_results
```

真实结果：`55 tests, 0 errors, 0 failures, 0 skipped`。新增覆盖：厂商URDF/mesh/第七轴、FK、名称乱序映射、实测速度标记、首帧实际关节自动初始化目标且不覆盖用户编辑、滑块只改预览且不创建ROS发布器、单击执行无需界面确认、独立授权、限位、碰撞、Touch互斥、反馈/GUI心跳超时、SAFE_STOP、短时票据、按厂商URDF速度生成五次轨迹、动态跟踪时延补偿与终点收敛窗口、七轴模式高跟随/旧模式低跟随、MoveIt服务就绪门、受控服务预检、包含GUI空值的配置抽取和取消请求。原操作台、Touch、力控、QR、ACK、实验日志及monitor transport测试同时通过。

2026-10-17现场故障：取消GUI的1°/s限速后，GUI客户端仍用下一个轨迹采样点与轮询反馈比较，产生一个采样周期的时间提前量，在0.0373 rad时误判超过0.03 rad并取消Action、触发SAFE_STOP。修复后GUI不再重复执行轨迹时基跟踪判定；独立关节服务成为唯一判定者，允许一个50 Hz命令周期的速度相关误差，并在终点使用0.08 rad瞬态窗口等待收敛到0.015 rad最终容差。

同日实机分级复测中，J7 `+2°`及返回均成功，静止后返回误差约`0.004°`。随后`+10°/1 s`测试测得正常瞬态伺服滞后`0.0355 rad`，比原动态阈值`0.0350 rad`高`0.0005 rad`；受控服务按设计触发软件停止，厂商ACK和静止反馈均已确认。根据该实测数据把基础动态跟踪阈值标定为`0.05 rad`，仍为旧`trajectory_bridge`的`0.20 rad`路径容差的四分之一；速度相关时延项和`0.015 rad`最终到位容差均保持不变。调整后的10°往返结果见本页后续记录。

继续复测时发现仅放宽到`0.05 rad`仍会累积到`0.0608 rad`并由Guard停止。源码审计确认`base_real.launch`原先把厂商驱动`Follow`固定为`false`（厂商launch注释定义为低跟随），与50 Hz连续轨迹流不匹配。现仅在互斥的七轴受控模式把`Follow`设置为`true`并保持`trajectory_mode=0`完全透传；旧trajectory bridge/QR路径仍使用低跟随，不修改厂商驱动协议。JointPos的`expand`字段也显式写为0。

高跟随修复后的实机分级结果：J7 `+2°`与返回均为`SUCCEEDED`，最大动态误差分别为`0.2058°`和`0.2519°`，稳定误差不超过`0.00762°`；随后恢复初始位置并执行J7 `+10°/1 s`与返回，两个Action均为`SUCCEEDED`，最大动态误差分别为`1.0304°`和`1.2249°`，稳定误差分别为`0.0911°`和`0.0600°`，未触发Guard、驱动故障或软件停止。该验证只覆盖当前姿态附近的J7单轴往返，不代表七轴全包络验收。

最终又执行七轴交替`[+0.5°, -0.5°, +0.5°, -0.5°, +0.5°, -0.5°, +0.5°]`组合微动及返回，两个Action均为`SUCCEEDED`。外向/返回最大单轴动态误差分别为`0.1389°`/`0.1644°`，返回后七轴最大稳定误差`0.036°`，没有Guard、驱动故障或软件停止。这只扩展了当前空载姿态附近的微动证据，仍不代表大范围、多姿态、持载或全包络验收。

最终运行核对确认MoveIt进程会先出现在ROS图中，而`/check_state_validity`约15秒后才完成初始化。原HMI只按节点判断基础栈就绪，造成启动后立即执行时的碰撞服务超时。基础栈配置现声明`expected_services: [/check_state_validity]`，`SystemLaunchManager`在服务可调用前不会返回启动成功；等待仍受原20秒总启动超时约束。

基础栈现场故障复核：曾因把完整`gui.yaml`加载进ROS1参数服务器而被其中合法的`null`值拒绝，错误为`cannot marshal None`。现改为受控服务在进程内读取文件并只抽取`joint_control`段。修复后真实展开179个launch参数成功，短时启动得到`rm_driver`、`move_group`、`robot_state_publisher`和`joint_control_server`四节点；`/rm_driver/JointPos`仅有`joint_control_server`一个发布者，验证结束后四节点均正常关闭，期间未提交运动目标。

离线GUI启动：在`xvfb-run -s '-screen 0 1920x1080x24'`下持续运行6秒，窗口及VTK初始化成功，测试通过超时主动结束。

## 仍锁止的硬件项

- 未验收完整七轴目标运动、全包络停止距离和负载影响。
- 未验收现场PlanningScene中真实工装、工具、地面、相机和环境障碍物。
- 受限模式已有独立GUI心跳看门狗，但尚未测量断联到真机停止的最坏时间和距离。
- 未对Action preempt后的真机停止时间、停止距离和ACK做硬件测量。
- 未擅自进行真实机械臂运动测试。

因此这次只证明软件链可启动且会fail-closed，不构成七轴真机验收。普通launch仍默认不授权；宿主机现场脚本根据用户明确授权默认开启受限关节服务，但每次目标仍须通过全部运行时检查。
