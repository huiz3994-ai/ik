# RM75-6FB 单 AprilTag 五目标定位

## 安全结论

本功能默认识别 `apriltag16h5`、ID `0`，五个 Tool 共用该标记。代码已实现检测质量审计、参考标定、五目标 SE(3) 计算、GUI 预览、MoveIt 两段规划和受控执行入口，但当前仓库配置会明确显示“坐标标定未完成”，并拒绝规划接近与真机执行。

拒绝不是软件缺陷，而是以下现场证据尚不存在：公共标记参考位姿、插入方向/距离、工具架及五个执行器碰撞几何、现场规划场景和逐项物理验收。现有约 11×11 px 标签观测也低于配置的 20 px 门限。

系统不会把图片标注的 30 mm 外框当作 PnP 边长。根据用户提供并确认的尺寸图，编码黑方块边长为 `22.50 mm`，因此 PnP 使用 `0.0225 m`，且 `marker_size_measurement_verified: true`。

## 实际调用关系与复用边界

单标记默认链路为：

```text
D435i RGB + CameraInfo
  → marker_node.py（OpenCV AprilTag、PnP、同步时间戳 TF、质量门）
  → /qr_task/marker_observation（仅 ID=0）
  → five_target_server.py（标定版本、五目标 SE(3)、快照冻结）
  → /qr_task/five_targets/command
  → TaskManager / ROSInterface 后台线程
  → 现有六页 PyQt5 上位机
  → MoveIt/FCL（仅在场景与接近几何确认后规划）
  → 现有 trajectory_bridge（仅在全部执行门通过后执行）
```

复用内容：相机启动、图像/内参同步、PnP、TF2、手眼静态 TF、MoveIt、trajectory bridge、急停、控制源互斥和厂商工具电压 ACK 接口。GUI 不发布 `/rm_driver/JointPos`。新流程不使用旧四标记 `PICK/RETURN` 坐标，而是在新的公共标记五目标服务中完成 J7 观察、视觉定位、到位、24V 吸附和退回观察位。

旧多标记兼容模式仍保留在 `rm75_qr_task/launch/qr_task_real.launch` 与 `config/markers.yaml`。新的 GUI 默认启动 `single_marker_five_targets.launch`。

## 坐标与计算

内部只使用米、xyzw 四元数和 4×4 右手刚体变换，并检查旋转正交性与行列式：

```text
T_B_M = T_B_L × T_L_C × T_C_M
T_M_L_i = inverse(T_B_M_ref) × T_B_L_i_ref
T_B_L_i_new = T_B_M_current × T_M_L_i
```

姿态不进行欧拉角相加。控制器源码把 position 从微米换算成米、Euler 从毫弧度换算成弧度，并使用 `tf2::Quaternion.setRPY(rx, ry, rz)`，即固定轴 XYZ RPY（矩阵 `Rz·Ry·Rx`）。2026-10-13 的只读审计在一个静止姿态得到控制器反馈与 `base_link→Link7` FK 的位置差 0.168 mm、姿态差 0.0713°。用户进一步确认五组数据是官方示教器取得的活动末端位姿。本基础 MoveIt 链以 `Link7` 为末端，故本模式明确把五组数据作为 `base_link→Link7` 参考目标，不再要求一个额外的 TCP 标定。

五个原始示教值转换为以下 `base→Link7` 参考位姿：

| Tool | XYZ m | quaternion xyzw |
|---|---|---|
| 1 | `[-0.120806, 0.254971, 0.217749]` | `[0.732625766, 0.679362098, 0.040536007, 0.009135591]` |
| 2 | `[-0.189102, 0.258388, 0.213392]` | `[0.690129296, -0.723561817, 0.006587582, 0.011681451]` |
| 3 | `[-0.119581, 0.326464, 0.211952]` | `[0.713367019, 0.700668596, -0.012953234, 0.001796596]` |
| 4 | `[-0.189641, 0.328648, 0.211018]` | `[0.649410005, -0.760328244, 0.012857723, 0.001511840]` |
| 5 | `[-0.249211, 0.116644, 0.177064]` | `[-0.693782871, 0.720117913, 0.004937691, 0.008434373]` |

当前没有 `T_B_M_ref`，因此五个 `T_M_Link7_i` 和当前视觉修正目标仍“不可计算”；完成一次“标定工具架”后即可生成。本模式不读取、不校验、也不要求 `Link7→TCP`；持久化数据仅为兼容旧 schema 自动写入单位变换。

## 配置与持久化

- 检测配置：`rm75_qr_task/config/single_marker.yaml`
- 五目标/安全模板：`rm75_qr_task/config/single_marker_five_targets.yaml`
- 默认运行时标定：`/root/.ros/rm75_qr_task/single_marker_five_targets.yaml`
- 旧多标记配置：`rm75_qr_task/config/markers.yaml`

运行时标定包含 schema、版本、marker family/ID/22.5 mm 编码方块尺寸、`base→marker` 参考、手眼变换、五个参考 Link7 位姿、五个 `marker→Link7`、检测质量、数学回代与独立物理验证状态。为保持 ROS JSON 合约兼容，内部部分字段仍命名为 `tcp`，但本配置中的 `tcp_frame` 明确为 `Link7`。写入采用同目录临时文件、`fsync` 和 `os.replace` 原子替换；加载时检查 schema、版本、frame、手眼变换和五个 slot 完整性。

手眼变换当前来自 `rm75_qr_task/config/handeye_calibration.yaml`：

```text
Link7 → camera_link
translation = [0.02418851387922378, 0.09074710310769298, 0.025447015109483433]
quaternion_xyzw = [-0.49047045364387526, -0.5138488336139114,
                   -0.49518343803705034, 0.5001914363419274]
```

marker 节点同时审计 `/tf` 与 `/tf_static` 中 `camera_link` 的发布者；发现多个发布者立即拒绝观测。

## ROS 接口

服务 `/qr_task/five_targets/command` 继续使用 `rm75_qr_task/TaskCommand` 合约，支持：

- `calibration_status`
- `calibrate_reference`
- `detect`
- `compute_targets`
- `preview`
- `plan`
- `execute_selected`
- `cancel` / `stop`
- `status`

`slot_id` 为 1～5。`execute_selected` 还要求请求的 `execute=true`。

只读/预览输出：

- `/qr_task/debug_image`：ID、角点框和像素尺寸叠加图
- `/qr_task/vision_status`：角点、像素尺寸、重投影误差、稳定性与 TF 发布者
- `/qr_task/five_targets/status`
- `/qr_task/five_targets/targets`
- `/qr_task/five_targets/tcp_poses`
- `/qr_task/five_targets/selected_tcp`
- `/qr_task/five_targets/selected_link7`

## 首次标定

1. 已确认尺寸图中的编码黑方块为 22.50 mm；30 mm 外框不参与 PnP。
2. 已确认五组数据为官方示教器末端位姿，并按当前 MoveIt `base_link→Link7` 末端使用。
3. 确认只有一个节点发布 `Link7→camera_link`。
4. 将机械臂置于安全观察姿态并保持静止；调整距离/分辨率，使 ID=0 达到至少 20 px 且稳定度通过。
5. 用现有点动把 J7 调到 RGB 相机能稳定看到 ID=0 的观察角度，在 GUI 点击“标定工具架”。服务同时捕获当前 J7 角度和带时间戳的稳定 marker，计算五个 `T_M_Link7_i`，数学回代后原子保存。
6. 重启并用 `calibration_status` 检查版本/frame/手眼一致。
7. 保持执行授权关闭，改变相机观察姿态，统计五个预测目标的重复性。
8. 物理检查通过后，仅在 `supervised_commissioning_authorized:=true` 的监护验收模式下点击“确认现场验收”，经二次弹窗确认后原子写入标定文件。该操作不运动、不通电，程序不会自动把数学回代当作现场通过。

## 启动和操作

```bash
docker start rm75_noetic
docker exec -it rm75_noetic bash
source /opt/ros/noetic/setup.bash
cd /root/rm75_ws
catkin build rm75_qr_task rm75_sampling_gui --no-status
source devel/setup.bash
roslaunch rm75_sampling_gui gui.launch
```

宿主机现场脚本分为互斥的两种控制路径：默认 `RM75_CONTROL_PATH=joint_control` 启动 `/joint_control_server`，可同时启动 QR 做识别/标定/预览；真机自动取工具使用 `RM75_CONTROL_PATH=qr_pickup`，启动 `/trajectory_bridge` 并关闭七轴受控服务。切换前必须先在界面停止旧基础栈并退出旧 HMI。

推荐顺序：启动 RM75 基础栈 → RGB 相机 → Mark/QR 任务 → 识别公共二维码 → 选择 Tool1～Tool5 → 预览目标位姿 → 规划 → 现场验收并开启执行授权 → “自动取选中工具”。所有 ROS 耗时调用经现有线程池执行，不阻塞 Qt 事件循环。

本流程按用户要求不建立工具架/工具环境碰撞几何，不以 `collision_scene_verified` 作为执行门；MoveIt 仍检查机器人自碰撞、关节限位和轨迹可规划性。自动执行顺序为：发送 0V 并等待本次新鲜 ACK → 仅把 J7 移到首次标定时记录的观察角度 → 停稳后等待新的稳定 ID=0 图像 → 重新计算目标 → MoveIt 到达 Link7 示教点 → 核对末端误差 → 发送 24V 并等待本次新鲜 ACK → 退回取工具前的观察关节状态。真实执行仍要求其余 `verifications`、标定物理验证、实时关节反馈、唯一命令源、`execution_enabled`、QR 授权和 trajectory bridge 授权；任一不满足均拒绝。

## 测试

```bash
source /opt/ros/noetic/setup.bash
cd /root/rm75_ws
source devel/setup.bash
QT_QPA_PLATFORM=offscreen catkin run_tests rm75_qr_task rm75_sampling_gui --no-status
catkin_test_results build
```

测试覆盖原位回代、刚体平移、绕 XYZ 旋转、±π、非零 TCP、五 slot、错误 ID、过期视觉、损坏/版本不匹配标定、原子写入，以及原 QR/GUI/线程/控制源/急停回归。无硬件测试不发送运动和电压命令。
