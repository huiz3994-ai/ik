# rm75_sampling_gui

> 单公共 AprilTag 引导五目标定位的配置、标定、安全边界和验收结果见
> [SINGLE_MARKER_FIVE_TARGETS.md](docs/SINGLE_MARKER_FIVE_TARGETS.md) 与
> [validation/SINGLE_MARKER_FIVE_TARGETS.md](docs/validation/SINGLE_MARKER_FIVE_TARGETS.md)。

高原寒区采样机器人现场操作站（v0.6）。提供操作台、机器人状态、实时力、视觉、故障诊断、参数六个页面，以及实验数据记录。机器人状态页包含RM75-6FB三维数字孪生、七轴离线目标预览和有人值守的受限真机控制。PyQt进程和滑块回调从不发布`/rm_driver/JointPos`；真机命令只能经独立进程`joint_control_server.py`的Action、安全检查和看门狗后发布。本项目没有修改`rm_driver`、Touch、力控、QR或视觉算法。

## Web 控制平台（推荐新入口）

本包现已增加 React/TypeScript + Three.js 前端和 FastAPI 后端；旧 PyQt 入口完整保留作为
回退。生产前端由后端统一提供，默认只监听本机：

```bash
# 第一次安装隔离的 Python Web 依赖
docker exec rm75_noetic bash -lc 'python3 -m venv --system-site-packages /root/rm75_ws/.venv-rm75-web && /root/rm75_ws/.venv-rm75-web/bin/pip install -r /root/rm75_ws/src/rm75_sampling_gui/web/backend/requirements.txt'

# 构建前端（源码更新后执行）
cd /home/zjh/project/RealMan/rm75_ws/src/rm75_sampling_gui/web/frontend
npm install && npm run build

# 唯一正式入口；直接打开统一控制网页
/home/zjh/project/RealMan/rm75_ws/src/rm75_sampling_gui/scripts/start_sampling_web.sh

# 仿真服务启动后执行可重复的黑盒验收
docker exec rm75_noetic bash -lc '/root/rm75_ws/.venv-rm75-web/bin/python \
  /root/rm75_ws/src/rm75_sampling_gui/scripts/validate_web_simulation.py'

```

启动脚本不再提供模式菜单、终端确认或环境变量授权选择，直接启动统一网页并开放 Z 力位、
有人值守七轴、二维码取物、Tool5 钻取和采样硬件功能。统一入口首次启动默认装载七轴关节
控制链，但不会发送关节目标；用户点击具体功能时，后端自动停止网页拥有的旧子系统、切换
唯一运动命令链并按依赖重启所需子系统。外部节点、非空闲任务或失败的重启会安全拒绝，
不会同时运行冲突的运动命令源。旧
`start_sampling_web_realtime.sh` 暂时仅作为兼容包装保留，并转交到同一入口；非交互兼容
调用也会进入相同的统一模式和默认七轴路径。
统一入口还会等待上一实例的 Web 进程和 `roslaunch` 完全退出，避免重启时新实例复用即将被
旧实例关闭的 ROS Master。

网页中的七轴路径启用的是带反馈、碰撞检查、独占租约和失联看门狗的 50 Hz 有人值守控制链，
不是硬实时 RTOS。启动时基础栈会按默认七轴路径带起，但不会发送关节目标；浏览器中仍需
“获取控制权”、同步实际关节并显式执行目标。若 8088 已有观察模式实例，先在原终端按
`Ctrl+C` 正常退出，再运行真机入口。

Web 架构、PyQt 对照、API 和诚实验收边界见
[`docs/web/ARCHITECTURE.md`](docs/web/ARCHITECTURE.md)、
[`docs/web/MIGRATION_MAP.md`](docs/web/MIGRATION_MAP.md)、
[`docs/web/API.md`](docs/web/API.md) 和
[`docs/web/VALIDATION.md`](docs/web/VALIDATION.md)。浏览器控制租约必须由活跃 WebSocket
持续续期；页面关闭后不会由后端伪造或无限续期 ROS GUI 心跳。

### 路径示教与通用轨迹

Web 导航中的“路径示教”按操作员点击逐个记录新鲜 `/joint_states` 七轴实际角度和
`base_link → Link7` TF 位置/四元数，不进行自动采样。点位可选择、删除、上移、下移和清空；
任何点位或速度/加速度比例修改都会清除旧密集轨迹并标记 `STALE`。

“生成平滑轨迹”使用关节空间分段五次 Hermite 插值。单调相邻段采用非零调和平均通过速度，
方向反转的关节在该关键点速度归零，所有关键点共享连续速度和零结点加速度，因此位置、速度、
加速度为 C2 连续，同时每个原始七轴示教向量被原样写入密集轨迹。生成器根据 URDF 速度限制、
配置的七轴加速度限制及页面比例迭代放大时间，并对全部密集点检查关节限位和当前 MoveIt
碰撞场景。

轨迹以版本化 `RM75Trajectory` JSON 保存，默认目录为
`~/.ros/rm75_sampling_gui/trajectories/`，可由 `trajectory_library.directory` 调整。文件使用
稳定 `trajectory_id` 而不是显示名称，采用同目录临时文件、`fsync` 和原子替换。其他任务可
通过 `POST /api/v1/trajectories/{trajectory_id}/execute`（等价于服务层
`execute(trajectory_id)`）复用，不需再次实现插值。真实执行继续走
`TaskManager → JointControlAdapter → /rm75_sampling_gui/follow_joint_trajectory`，进度由实际
关节反馈匹配密集轨迹得到；停止、控制租约、SAFE_STOP 和竞争命令源保护保持不变。

受控 Action 服务与 SIMULATION 都使用保存的速度、加速度重建同一分段五次多项式，保留
结点通过速度；旧的仅位置 Action 仍兼容。进度从当前段沿实际反馈匹配，时间只限制可匹配的
上界，避免回返轨迹跳到未来段。停止事件贯穿排队、校验、Action 等待和执行阶段。
创建时重复 ID 返回冲突，不会覆盖已有路径；覆盖保存使用修改接口，参数未变化时保留 READY。
参数尚未保存或重新生成时，页面禁止执行旧轨迹。
验收命令、审查修复与实际结果见 [轨迹验收记录](docs/web/WAYPOINT_TRAJECTORY_VALIDATION.md)。

“五工具配置”页可编辑 Tool1–Tool5 的绝对 `base_link → Link7` 位姿（位置 m、固定轴
XYZ RPY rad）和垂直抬升高度，并原子持久化到 `config/tools.yaml`。确认执行前必须先通过
MoveIt Dry Run；执行顺序为预抓取 `Z+H`、保持姿态严格沿基坐标 Z 下降、现有 24V 取物
动作及新 ACK、保持姿态严格抬升 H、原位保持。配置或起点在 Dry Run 后发生变化会拒绝
执行。项目没有独立物体在位传感器，因此页面只显示“运动完成、取物未独立确认”，不会
把电压 ACK 冒充抓取成功。

网页导航将目标来源明确拆成两个互不混用的功能：**二维码取物**使用公共二维码 ID 0、
现有手眼/机架标定和五槽位变换，提供识别、计算、预览、Dry Run、确认取物、归还和终止；
**自主写点取物**只读取 `config/tools.yaml` 中人工填写的绝对位姿和 Lift Height。两者共用
MoveIt、轨迹桥、末端电压 ACK、急停及互斥控制安全链，但不会把自主写点覆盖成二维码目标，
也不会让二维码流程读取自主写点位姿。

开发验证可显式使用 `start_sampling_web.sh --mode simulation`。SIMULATION 与 OFFLINE 不同：OFFLINE 是静态只读检查；SIMULATION
提供内存关节动力学、六维力、TCP、公共 AprilTag 0 和合成 RGB 帧，并允许验证完整的
浏览器独占租约、命令白名单、急停锁存与恢复。该模式不调用 `ROSInterface.connect()`、
不发布控制心跳，也不创建 `/rm_driver/JointPos`，页面顶栏持续显示
`SIMULATION / 无物理输出`，不能与任何真实执行授权同时启用。

`z_force` 是独立、互斥的 Touch 控制路径。基础栈不会启动
`/trajectory_bridge` 或 `/joint_control_server`；网页“实时力”页的“启动 Z 力位 + 力反馈”
加载 `rm75_touch_z_force_haptic_real.yaml`。灰键在自由空间只改变机器人 Z，X/Y 和末端姿态
保持启动瞬间的值；检测到接触后 Z 由力外环接管。当前保守目标力为 -10 N、Z 速度上限
2 mm/s、接触后最大 Z 行程 10 mm，Touch 最终力向量上限 0.5 N。停止前必须松开灰键；
紧急情况使用实体急停或页面软件急停。该入口仍需完成现场低力验收，不能把软件限值视为
碰撞认证。

## RM75-6FB三维数字孪生与七轴交互

- `model/robot_model.py`运行时展开厂商`rm_75_6fb_description.urdf.xacro link7_type:=Link7_6fb`，自动读取`joint1..joint7`的origin、axis、limit、velocity和visual mesh。第七轴使用厂商分支中的0.1612 m几何与`Link7_6fb.STL`，界面没有复制一套手写DH/几何参数。
- `Robot3DWidget`用容器已有VTK嵌入PyQt5，实际模型为不透明浅色，离线目标为青色半透明；鼠标支持旋转、缩放、平移，并提供正视、侧视、俯视、复位、模型叠加和坐标系开关。显示`/joint_states`源时间戳/延迟，反馈过期时实际模型转为红色半透明并标记非实时。
- 已调查Python RViz：Noetic容器里的`rviz.VisualizationFrame`可导入，但RViz RobotModel依赖ROS TF，无法在不建立第二套冲突frame的情况下同时显示真实模型和独立离线目标。VTK方案可直接复用URDF STL和FK、无ROS也能预览；正式碰撞检查仍调用MoveIt/FCL的`/check_state_validity`，没有自行近似碰撞算法。
- `/joint_states`严格按名称映射，显示角度使用度，内部均为弧度。只有消息携带完整有限的velocity数组才标为“实测”；缺失时显示`—`，不会把差分估计伪装为驱动测量。
- 滑块、绝对目标框以及按住`+/-`只更新离线目标，绝不自动执行。点动失焦、窗口隐藏或按键释放会立即停止目标递增。可用“同步实际关节”把目标对齐到最新实测值。

### 真机关节控制安全门

`gui.launch`和配置文件仍默认锁止真机执行。宿主机现场入口`start_sampling_station.sh`根据用户明确授权，默认选择“有人值守受限七轴模式”：

```text
JOINT_CONTROL_EXECUTION_AUTHORIZED=true
TRAJECTORY_EXECUTION_AUTHORIZED=false
SUPERVISED_COMMISSIONING_AUTHORIZED=true
```

这个模式不使用旧`trajectory_bridge`。基础栈只启动一个命令源`/joint_control_server`，GUI通过`/rm75_sampling_gui/follow_joint_trajectory`提交已预览目标。七轴服务以50 Hz发送连续五次轨迹，因此该互斥模式将厂商驱动设置为`Follow=true, trajectory_mode=0, radio=0`（高跟随完全透传）；旧trajectory bridge/QR路径仍保持`Follow=false`。HMI不仅等待基础节点，还会等待MoveIt `/check_state_validity`服务真正可调用后才报告“启动完成”，避免节点已出现但碰撞服务仍初始化时误执行。若用普通`roslaunch rm75_sampling_gui gui.launch`启动，以上两个七轴授权仍为false。

“执行七轴目标”采用单击执行：不再需要单独点击安全检查、勾选确认或通过确认弹窗；TaskManager会在同一个后台任务中自动完成检查并立即消费短时票据。按现场要求已取消“目标相对实际关节的单次最大角度差”、GUI额外的1°/s速度上限，以及额外的加速度/jerk拒绝阈值，但不会取消厂商URDF机械限位、0.02 rad限位余量和URDF各关节速度上限。完整检查顺序为：七维有限值 → URDF限位和余量 → 新鲜关节反馈 → SAFE_STOP未锁存 → Touch/垂直力控/rm_control/调试执行器及QR任务互斥 → 沿整条路径逐点调用MoveIt/FCL → 独立关节授权和受限模式授权 → Action服务存在 → 2秒短时票据。轨迹仍使用连续五次时间律，并按厂商URDF各轴速度上限计算时间；独立服务会再次核验URDF速度、碰撞、控制源和驱动状态。

独立服务要求GUI心跳小于0.35 s、关节反馈小于0.15 s；取消、心跳丢失、反馈丢失、跟踪超差、驱动错误或互斥控制节点出现时发布已有软件急停，并等待本次命令后的新停止ACK及静止反馈。正常关闭期间有活动任务时仍禁止退出。软件急停不能替代实体急停；这些限制是投产前的保守工程值，并非完整真机安全认证。

## 控制链

- Touch：普通遥操作启动 `rm75_touch_teleop/teleop.launch`；Web `z_force` 路径复用该节点的既有 Z 力位混合与触觉渲染，只开放 Z 平移并锁定 X/Y/姿态；“独立垂直下压力控”仍启动 `rm75_vertical_press/vertical_press_real.launch`。
- 工具识别：订阅 `/qr_task/vision_status`，仅接受按钮按下后产生的稳定 Mark 结果，再按 `config/tools.yaml` 将 `slot_id` 映射为 Tool1–Tool5。
- 取放：调用 `/qr_task/command`（`rm75_qr_task/TaskCommand`）。MoveIt、碰撞场景、24V/0V 取放时序及持物/释放确认仍由 `rm75_qr_task` 负责。
- 电压：发布 `/rm_driver/SetToolVoltage`，等待本次命令之后的 `/rm_driver/Set_Tool_Voltage_Result`。协议为 24V→3、0V→0。
- 力控显示：订阅 `/rm75_vertical_press/{force_z,filtered_force_z,force_setpoint,active,control_mode,stop_reason}`。GUI 不绕过 Touch 的按键使能逻辑。
- 安全：软件急停发布 `/rm_driver/Emergency_Stop` 并等待新的 `/rm_driver/Set_Arm_Stop_Result`；恢复发布 `/rm_driver/Clear_System_Err` 并等待新的 `/rm_driver/System_En_State_Result`，然后调用 QR 任务恢复接口。恢复不会生成限位脱困运动。
- Web“末端位置控制”提供 `base_link` 平移和 `Link7` 局部旋转的按住点动；末端参考不是已标定工具 TCP。接口、安全限制和停止确认见 `docs/web/CARTESIAN_JOG.md`。

各功能不再受固定任务状态链约束。按钮根据真实ROS节点、HMI拥有的控制进程、工具选择和执行授权独立启用；操作状态只用于显示和日志。底层控制异常仍会停止GUI管理的控制launch并锁存`SAFE_STOP`，恢复检查仍不产生运动。

## 默认安全门

`gui.launch` 和 `gui_real.launch` 默认均不授权真实运动。`config/tools.yaml` 中五个工具的 `enabled`、`calibrated` 和几何 `verified` 默认均为 `false`。现有 `rm75_qr_task/config/safety.yaml` 也仍有未通过的真机验证项。因此默认入口只用于界面、识别链和规划联调。

不要为了通过检查而直接修改这些值。必须先完成实测标定、碰撞场景、负载、持物反馈、停止距离和单一命令源验收。软件急停不能代替物理急停。

## 构建和测试

```bash
docker start rm75_noetic
docker exec -it rm75_noetic bash
source /opt/ros/noetic/setup.bash
cd /root/rm75_ws
catkin build rm75_sampling_gui --no-status
source devel/setup.bash
catkin run_tests rm75_sampling_gui --no-status
catkin_test_results build/rm75_sampling_gui/test_results
```

## 启动

推荐直接在宿主机运行一键脚本。它会启动容器、临时授权X11并且只启动HMI；基础栈、相机、QR、Touch和力控均在界面内按需启动：

```bash
/home/zjh/project/RealMan/rm75_ws/src/rm75_sampling_gui/scripts/start_sampling_station.sh
```

脚本默认 `RM75_CONTROL_PATH=joint_control`，用于七轴点动；此模式下 QR 仍可启动并完成识别、标定和预览，但自动取工具保持锁止。真正执行公共标记取工具时，必须先在界面停止已启动的基础栈并退出旧 HMI，再用互斥的 QR 路径启动：

```bash
RM75_CONTROL_PATH=qr_pickup \
  /home/zjh/project/RealMan/rm75_ws/src/rm75_sampling_gui/scripts/start_sampling_station.sh
```

该模式启用 `/trajectory_bridge`、QR 执行和工具电压授权，同时禁用 `/joint_control_server`。`joint_control` 与 `trajectory_bridge` 同时授权会在启动时被拒绝。

界面中的推荐顺序为“RM75基础栈 → RGB-D相机 → Mark/QR任务”。Touch遥操作和力控继续由对应任务按钮启动。每个停止按钮只会停止当前HMI创建的roslaunch，发现外部同名节点时会拒绝重复启动，也不会误杀外部进程。

现场D435i当前按USB 2.1枚举，因此界面的相机按钮默认调用本包`d435i_rgb.launch`，只开启424×240@6Hz RGB用于应急监看，关闭深度、点云和IMU以降低带宽。需要正常画质或QR深度链时应先将相机接入确认的USB 3.x端口，再单独验收完整`camera_real.launch`；不要把RGB-only监看误当成完整RGB-D验收。

相机按钮会在启动前仅对配置的D435i USB VID/PID执行解绑/重绑恢复，随后必须从`/camera/color/image_raw`收到非空首帧才报告启动成功。只有节点名而没有图像数据会判定失败并回收该launch，避免界面显示“运行中”但画面为空。

也可以在已经具备X11权限的容器终端中直接启动安全模式：

```bash
source /opt/ros/noetic/setup.bash
cd /root/rm75_ws
source devel/setup.bash
roslaunch rm75_sampling_gui gui.launch
```

`gui.launch`与`gui_real.launch`现在都只启动HMI，不会在界面出现前隐式启动QR或机器人节点。

真机执行只有在所有物理验收项和 `tools.yaml` 标定都已签字通过后，才同时设置：

```text
execution_enabled:=true
qr_execution_authorized:=true
qr_safety_config:=<经审核的安全配置绝对路径>
```

只打开一个开关仍会被拒绝。界面“放回工具”会继续调用现有 `rm75_qr_task`，其到位、0V和释放确认逻辑不变。24V与0V按钮不再依赖工具流程状态，但每次命令仍必须收到对应的新ACK。

一键脚本优先使用 `RM75_CONTROL_PATH` 选择互斥命令路径，也保留各独立环境变量用于审核后的精确覆盖。默认 `joint_control` 模式开启受控七轴执行，QR/MoveIt取工具仍关闭。如需低层覆盖可显式传入：

```bash
EXECUTION_ENABLED=true QR_EXECUTION_AUTHORIZED=true \
  JOINT_CONTROL_EXECUTION_AUTHORIZED=false \
  TRAJECTORY_EXECUTION_AUTHORIZED=true \
  /home/zjh/project/RealMan/rm75_ws/src/rm75_sampling_gui/scripts/start_sampling_station.sh
```

真实Touch授权不会自动授权MoveIt轨迹桥。`trajectory_execution_authorized`默认保持`false`，用于确保Touch联调时trajectory bridge不执行轨迹；工具取放阶段必须经过独立现场验收后才能单独开放。

七轴离线预览无需任何执行授权。现场脚本当前默认`JOINT_CONTROL_EXECUTION_AUTHORIZED=true`、`SUPERVISED_COMMISSIONING_AUTHORIZED=true`和`TRAJECTORY_EXECUTION_AUTHORIZED=false`，因此使用独立受控服务而不是旧轨迹桥。需要Touch或垂直力控时，应停止基础栈并以`JOINT_CONTROL_EXECUTION_AUTHORIZED=false`重启；界面不会让两个命令源同时运行。

HMI启动QR识别时会同时发布现有的`Link7 → camera_link`手眼标定静态TF；D435i驱动只负责`camera_link`以下的相机光学TF。若已有其他节点发布同一手眼TF，应先停止重复发布者，确保该子树只有一个父节点。

## 分层测试建议

1. 离线：运行本包测试，检查状态机、ACK 新鲜度、未授权拒绝和异常 `SAFE_STOP`。
2. ROS 合约：用 mock publisher/service 验证 ACK 超时、ACK=false、旧 ACK、Mark 超时和 QR 服务拒绝。
3. 规划联调：保持两个执行授权为 false，检查五个 Mark 映射、MoveIt preview 和界面状态。
4. 台架：机械臂断使能，逐项验证 24V/0V ACK、独立持物/释放反馈、急停和清错。
5. 受限真机：低速、空载、监护人和物理急停就位，按单步方式验证每个工具的 PICK/RETURN，再验证 Touch 力控；最后才做端到端流程。

当前已知边界：现有系统没有供 GUI 使用的“限位自动脱困”ROS 服务，所以恢复只清错和恢复任务状态。七轴模式由launch排除`trajectory_bridge`，并拒绝Touch/力控/`rm_control`；切换功能必须先停止并重启基础栈。尚未完成全工作空间、负载、真实场景、停止距离和实体急停链的系统验收。

2026-10-17当前姿态附近的实机分级验证已完成J7 `+2°`/返回和`+10°/1 s`/返回，四个Action均为`SUCCEEDED`。10°往返最大动态误差为`1.0304°/1.2249°`，稳定误差为`0.0911°/0.0600°`。随后完成七轴交替±0.5°组合微动与返回，两个Action同样为`SUCCEEDED`，最大单轴动态误差`0.1644°`、返回稳定误差最大`0.036°`；均未触发Guard或驱动故障。GUI收到第一帧新鲜关节反馈时会自动把预览目标同步到实际姿态一次，避免未编辑轴保留零目标；用户开始编辑后不再自动覆盖。该结果只证明当前空载姿态附近的小范围控制，不能外推为七轴全包络验收。自动化回归为`55 tests, 0 errors, 0 failures, 0 skipped`。

## 现场操作站页面

| 页面 | 数据和显示 |
|---|---|
| 机器人状态 | 有界XML-RPC查询Master和配置的节点；七轴角度与URDF最近限位距离（度）；TF `base_link → Link7` 的XYZ（m）与RPY（度）；持有/选择工具、电压ACK |
| 实时力 | Qt实时绘图，默认保留600点/约60秒；Fx/Fy原始补偿值，Fz优先取新鲜filtered_force_z，其次force_z，力控未运行时回退到Six_Force原始Fz；目标力、当前力、目标减当前的误差 |
| 视觉 | 自动发现RGB Image topic，运行期间相机上线也可接入；支持rgb8/bgr8/rgba8/mono8；Tool1–Tool5的Mark ID、检测状态、基座位姿 |
| 故障诊断 | 通信、力反馈、工具、QR的OK/ERROR；超时、ACK失败、SAFE_STOP、sensor lost分类；相同持续故障不重复刷屏 |
| 参数 | 只读展示配置映射指定的YAML字段及来源文件，不写ROS参数；采样时间未配置时明确显示“未配置” |

节点OK表示ROS节点可达，不代表控制器运动已验收。角度/TF/图像过期后显示不可用；限位没有URDF依据时显示空缺。Link7是法兰参考，不冒充已标定工具TCP。

原始Six_Force和力控Fz采用不同符号/处理约定，界面和日志明确区分。新鲜force_setpoint不会让已经断流的测量Fz变为正常。

## 新增只读ROS接口

| 接口 | 类型/用途 |
|---|---|
| `/joint_states` | sensor_msgs/JointState，按joint1–joint7名称映射；校验时间戳、完整性、有限值 |
| `/robot_description` | ROS参数，读取URDF上下限；不硬编码关节限位 |
| `/tf`、`/tf_static` | tf2订阅，读取base_link到Link7变换；不计算/发布控制目标 |
| `/rm_driver/UdpSixZeroForce` | rm_msgs/Six_Force，Fx/Fy/Fz及Mx/My/Mz分别写入Fx/Fy/Fz及Tx/Ty/Tz |
| `/qr_task/marker_observation` | std_msgs/String JSON，slot_id、stamp、frame_id、translation、quaternion_xyzw |
| `/qr_task/vision_status` | 继续使用已有质量/检测状态 |
| `/camera/color/image_raw` | 首选sensor_msgs/Image；不存在时发现名字含color/rgb的Image topic |
| `/rm_driver/Plan_State` | rm_msgs/Plan_State；执行期间新鲜的false触发停止 |
| `/rm_driver/ArmError`、`/rm_driver/SysError` | std_msgs/UInt16；执行期间新鲜非零值触发停止 |
| `/check_state_validity` | moveit_msgs/GetStateValidity；对关节路径采样调用MoveIt/FCL，只读检查 |
| `/rm75_sampling_gui/follow_joint_trajectory` | GUI向独立受控服务提交七轴目标；服务复核安全条件后成为唯一`JointPos`发布者 |
| `/rm75_sampling_gui/joint_control_heartbeat` | GUI运行心跳；超过0.35 s触发服务停止与ACK确认 |

原有三个Z向力话题、QR任务服务和电压/停止ACK接口保持使用。新增Monitor只有订阅和参数读取，不创建控制发布器。节点名、图像topic、wrench topic、刷新时间、曲线长度、TF参考帧都由 `config/gui.yaml` 配置。

## 实验数据

默认写到本包下 `experiments/YYYYMMDD_NNN/`，用原子目录创建避免覆盖；可通过 `experiment.directory` 指定可写绝对路径。目录在任务开始时创建。

- `robot_state.csv`：UTC Unix时间戳、joint1–joint7（rad）、XYZ（m）、RPY（rad）、关节/TF源时间戳、新鲜度标记。
- `force.csv`：时间戳、原始Fx/Fy/Fz/Tx/Ty/Tz、单独的force_z/filtered_force_z/force_setpoint、新鲜度。Six_Force没有Header，使用接收时刻/快照时间，不伪造传感器时间戳。
- `task.log`：状态变化、错误及正/负ACK；SAFE_STOP之后晚到的停止ACK继续追加到同一实验日志。
- `config.yaml`：GUI参数、五工具完整配置、启动授权、实际读取的遥操作/力控YAML内容与路径、单位/记录说明。它是配置文件快照，不声称包含外部launch的所有运行时覆盖。

记录频率默认10Hz，属于现场监控采样，不是200Hz原始传感器无损存储。单个实验从任务开始至返回IDLE或SAFE_STOP；直接从遥操作进入Mark识别保持同一实验。手动停止遥操作到IDLE视为结束。异步有界写队列避免磁盘写入阻塞GUI；磁盘错误/队列溢出明确显示，缺失或过期数据留空，不补零。结束时刷新并关闭文件。

## 无硬件启动与验证

在Noetic容器环境中：

```bash
source /opt/ros/noetic/setup.bash
cd /root/rm75_ws
catkin build rm75_sampling_gui --no-deps --no-status
source devel/setup.bash

# 不需要ROS master/相机/机械臂；不创建ROS控制发布器
rosrun rm75_sampling_gui sampling_gui.py --offline

# 已有ROS基础栈时，只启动HMI并接入现有节点
roslaunch rm75_sampling_gui gui_real.launch start_qr_task:=false

# 自动测试包含独立测试Master中的模拟消息，无真实硬件命令
QT_QPA_PLATFORM=offscreen catkin run_tests rm75_sampling_gui --no-deps --no-status
catkin_test_results build/rm75_sampling_gui/test_results
```

也提供 `station_offline.launch`（roslaunch自身会启动Master，但HMI使用offline模式）。默认真实执行仍关闭。旧 `ui.main_window` 导入保留兼容，主窗口实现迁移到 `gui/main_window.py`。

急停在调用ROS前立即锁存；它有独立工作线程，不排在普通任务后面。等待停止/任务返回期间也禁止关闭窗口。正常SIGINT/SIGTERM经过同一关闭检查。Touch启动改用独立roslaunch进程，解决Qt工作线程不能安装roslaunch信号处理器的问题，仍调用原launch与参数。

中文字体包含在 `assets/fonts/`，附版权和Apache-2.0许可证。无需修改容器系统字体。

新增/修改文件清单、测试结果和仅使用模拟数据的页面截图见 [本轮验证记录](docs/validation/FIELD_STATION.md)。
# Reference GUI additions

The Web HMI also includes the missing operator features from the ROS2 sampling
robot interface without replacing the RM75-specific workflows:

- joint/cartesian target tabs with IK-only preview and the existing guarded
  seven-axis execution path;
- persistent Cartesian presets in `config/poses.yaml`;
- persistent planner/speed selection in `config/ui_settings.yaml`;
- a Robot Control page for Touch teleoperation, one-click presets, force
  history, task stop controls, and tool-head 24 V/0 V commands.

Cartesian execution does not bypass the RM75 authorization gates. The IK
result is submitted to the same limit, freshness, ownership, collision, lease,
and trajectory checks used by the existing joint-control page.
