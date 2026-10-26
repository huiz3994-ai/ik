# RM75 + D435i 自动手眼标定

这个包为 ROS1 Noetic 下的 RM75-6FB、末端安装 D435i、固定 ArUco 标定板提供 eye-in-hand 自动标定。它复用 `easy_handeye` 的 MoveIt 运动生成、TF 采样和 OpenCV 手眼求解。

## 标定前

1. D435i 必须刚性安装在 `Link7` 上，标定过程中不可松动。
2. ArUco 板固定在工作台上，完整出现在彩色图像中；`marker_size` 是黑色方框的实际边长（米）。
3. 把 RM75 手动移动到一个安全的中心姿态，使相机正对标定板，并保证周围至少留出 `translation_delta` 的空间。
4. 退出遥操作、力控和其他机械臂控制节点；同一时间只能由 MoveIt 控制机械臂。
5. 核对实体型号。当前控制器已自报 `RM75-6FB`，此 launch 固定使用 `rm_75_moveit_config` 的 `Link7_6fb` 模型。

## 构建

`easy_handeye` 当前位于另一个工作空间，因此先 source 它：

```bash
source /opt/ros/noetic/setup.bash
source /home/zjh/project/Unitree/z1_ws/devel/setup.bash
cd /home/zjh/project/RealMan/rm75_ws
catkin config --extend /home/zjh/project/Unitree/z1_ws/devel
catkin build rm75_handeye_calibration
source devel/setup.bash
```

## 先做只读检查

如果 RM75/MoveIt 已启动，保持 `start_robot_stack:=false`：

```bash
roslaunch rm75_handeye_calibration rm75_d435i_handeye.launch \
  start_robot_stack:=false auto_start:=false execute_motion:=false \
  marker_id:=39 marker_size:=0.10
```

确认下列项目后再关闭该 launch：

```bash
rostopic hz /camera/color/image_raw
rosrun tf tf_echo camera_color_optical_frame camera_marker
rosrun tf tf_echo base_link Link7
rosservice list | grep rm75_d435i_eye_on_hand
```

## 自动标定

未启动 RM 驱动和 MoveIt 时，使用：

```bash
roslaunch rm75_handeye_calibration rm75_d435i_handeye.launch \
  start_robot_stack:=true arm_ip:=192.168.1.18 udp_ip:=192.168.1.10 \
  marker_id:=39 marker_size:=0.10 \
  auto_start:=true execute_motion:=true
```

已有 RM 驱动和 MoveIt 时，把 `start_robot_stack` 改成 `false`。脚本默认采集起始姿态加 17 个目标姿态，单个位姿不可达时跳过，少于 8 个有效样本则拒绝保存。结果由 `easy_handeye` 保存到：

```text
~/.ros/easy_handeye/rm75_d435i_eye_on_hand.yaml
```

标定后发布 `Link7 -> camera_link`：

```bash
roslaunch rm75_handeye_calibration publish_calibration.launch
```

标定和发布的结果是 `Link7 -> camera_link`。RealSense 驱动继续负责 `camera_link -> camera_color_optical_frame` 等设备内部 TF。发布标定 TF 时，不要再从 URDF 或其他 static publisher 发布同一个 `camera_link` child，否则会产生 TF 冲突。
