# RM75 Touch teleoperation

RM75 六维力到 Geomagic Touch 的可选触觉反馈、参数、安全门控与 commissioning
流程见 [docs/haptic_feedback.md](docs/haptic_feedback.md)。该功能默认关闭。

This package adapts the Geomagic Touch pipeline and the proven axis mapping in
the Unitree Z1 teleoperation code to the seven-axis RM75 model.

## Controls

- Hold the **grey** button to engage Cartesian translation. The current Touch
  position and RM75 position become zero-discontinuity anchors when it is pressed.
- Hold the **white** button independently to rotate the RM75 wrist. The current
  Touch quaternion and RM75 orientation become anchors when it is pressed.
- Releasing either button freezes that component at its last target. Commands
  stop when both buttons are released or after an input timeout.

Stop a launch with `Ctrl+C`, not `Ctrl+Z`. If a terminal was suspended or
closed and a control node remains live, clean up only the RM75/Touch stack with:

```bash
/root/rm75_ws/stop_rm75_touch.sh
```

The stop helper resumes a suspended roslaunch before requesting normal shutdown.
If teleoperation is still active, it requests `/rm_driver/Emergency_Stop` first.
It leaves unrelated shared camera/perception nodes running.

## Run

The combined launcher starts the Gazebo GUI, RM75 controllers, MoveIt backend,
Touch driver, and teleoperation node. It deliberately does not start RViz:

```bash
/root/rm75_ws/start_rm75_touch.sh
```

For an already-running RM75 simulation, the mapping-only launch remains
available (it does not open any interface):

```bash
roslaunch rm75_touch_teleop teleop.launch start_touch_driver:=false
```

Check the live chain with `rosrun rm75_touch_teleop check_touch_teleop.sh`.

The defaults target only RM75: MoveIt group `arm`, tip `Link7`, seven joints,
and `/arm/arm_joint_controller/command`. Parameters are in
`config/rm75_touch.yaml`.

The combined simulation first moves RM75 from its singular all-zero pose to
the official SRDF `forward` pose. Wait for this three-second move to finish,
then hold the grey button while moving the Touch. The imported
`geomagic_myhaptics/pubforce` node supplies independent position and orientation
anchor/increment topics. The legacy `v1.6_pubforce.launch` still starts
`pubforce_single`, whose white-button orientation path remains unavailable.

## Real RM75

The hardware launcher starts `rm_driver`, loads the RM75 kinematics, and starts
the Touch chain without Gazebo or RViz. Motion is deliberately disarmed by
default:

```bash
/root/rm75_ws/start_rm75_touch_real.sh \
  arm_ip:=192.168.1.18 udp_ip:=192.168.1.10
```

After checking the robot workspace, E-stop, network addresses, live
`/joint_states`, and Touch buttons, arm motion explicitly:

```bash
/root/rm75_ws/start_rm75_touch_real.sh enable_motion:=true \
  arm_ip:=192.168.1.18 udp_ip:=192.168.1.10
```

Real mode streams seven-joint commands to `/rm_driver/JointPos`, which is the
RM driver CAN-FD follow input. Its separate `rm75_touch_real.yaml` limits the
Touch scale to 0.5, Cartesian displacement to 0.10 m, and joint speed to
0.25 rad/s. The grey/white button behavior is identical to simulation.

For an RM75 fitted with the six-axis force sensor, use the dedicated model and
the host address configured on `enp3s0`:

```bash
# Connection/feedback check only; motion is disarmed.
/root/rm75_ws/start_rm75_touch_real_6f.sh
rosrun rm75_touch_teleop check_rm75_real_6f.sh

# Stop the check launch, clear the work area, then explicitly arm motion.
/root/rm75_ws/start_rm75_touch_real_6f.sh enable_motion:=true
```

This launcher uses the confirmed `RM75_6FB` URDF (`Link7_6fb`), robot address `192.168.1.18`, and UDP
destination `192.168.1.100`. It also requires fresh compensated external-wrench
feedback from `/rm_driver/UdpSixZeroForce`; Touch disengages if feedback becomes
stale or exceeds 35 N / 5 Nm. These thresholds are initial conservative values
and can be adjusted in `config/rm75_touch_real_6f.yaml` after payload testing.

## Soil sampling / 土壤采样

新增的可选 Z 力位混合控制使用原遥操作节点，普通配置默认关闭。
完整中文说明见 [采样力位混合控制使用说明](docs/采样力位混合控制使用说明.md)。

- Simulation: `/root/rm75_ws/start_rm75_sampling_sim.sh`
- Hardware (disarmed): `/root/rm75_ws/start_rm75_sampling_real.sh`
- Hardware commissioning requires verified wrench frame/polarity in the sampling YAML.
- The simulation soil node is an analytic sensor model, not a physical contact plugin.
