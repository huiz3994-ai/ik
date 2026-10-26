# 真机准备与联调（尚未实机验证）

用户目前没有真机，本目录交付代码和可执行入口，不声明真机抓取通过。

1. 控制器已确认机械臂为 RM75-6FB；核对所加载模型为 `Link7_6fb`，并继续核对七轴关节限位、零点和工具 TCP。
2. 用实际夹爪 CAD/尺寸替换测试工具几何，标定 `Link7 → grasp_tcp`；配置质量、负载和真实开口。六维力传感器包含在型号模型里，本任务未实现力控插补或力反馈抓取。
3. 安装 ROS1 兼容版本的 realsense2_camera；禁止误装只支持 ROS2 的新分支。检查设备序列号、USB3、RGB/深度对齐、CameraInfo、PointCloud2 时间戳。
4. 眼在手上标定输出 `Link7 → camera_link`。把平移/旋转写入 launch 参数，旋转为 URDF RPY（弧度）；相机内参从 CameraInfo 读取，不能硬编码 D435i 通用数值。
5. 接通厂商 driver，校验 Arm_IP/Udp_IP/代际设置和七轴反馈。默认入口不自动连接设备，需要明确 driver:=true。
6. 根据夹爪型号实现 `/gripper/object_detected`，约定 ≥10 Hz、新鲜 Bool、关闭后的真实物体存在反馈。必须排除“仅收到命令就发布 true”的实现。
7. 先保持 execute:=false，运行 preflight，使用 RViz 查看模型、TF、点云和 OctoMap，验证静态障碍/桌面尺寸。
8. 验证 `/rm_driver/Emergency_Stop` (`std_msgs/Empty`) 的停止行为及 `/rm_driver/Set_Arm_Stop_Result`。测量命令、ACK、实际停止延迟；测试中断通信、过期关节、取消 Action。新 bridge 不与旧 rm_control 同时启动。
9. 完成上述校准后，在独立 task YAML 中设 `hand_eye_calibrated/tool_verified/stop_verified/gripper_feedback_verified: true`，再使用 execute:=true。标志只是部署确认，不替代实际测试。
10. 从空载、小范围、低速、单障碍开始。验证 top/side 的可达性，再测试目标抓取、抬升、动态障碍、点云断流和夹爪失败。记录日志和实际停止距离。

启动（容器内，设备尚未接入时可以只启动规划配置）：

```bash
source /opt/ros/noetic/setup.bash
source /root/rm75_ws/devel/setup.bash
roslaunch rm75_autonomous_grasp real.launch execute:=false camera:=false driver:=false
```

设备到位后：

```bash
roslaunch rm75_autonomous_grasp real.launch \
  camera:=true driver:=true execute:=false \
  arm_ip:=192.168.1.18 udp_ip:=192.168.1.10 \
  camera_xyz:='测得的三个平移值' camera_rpy:='测得的三个弧度值' \
  tcp_xyz:='测得的三个平移值'
```

示例中的中文占位值必须替换成数字。任务通过 `/grasp` Action 启动，默认不会自动抓取。相机和夹爪未知时必须保持执行关闭。规划模式没有真实 joint_states 时不会伪造反馈，因此 MoveIt 可能等待当前状态，这是正常的。
