# RM75 adaptations from z1_examples

The original Z1 Touch sources requested for reference are preserved under
`reference/`. They are not built because they hard-code the Z1 six-axis
`manipulator` group, `link00`, Unitree messages, and `/goal_joint_positions`.

`rm75_get_end_effector_pose` is the corresponding RM75 implementation using
MoveIt group `arm`, tip `Link7`, and seven joints. The live Touch-to-RM75 IK and
trajectory mapping is implemented by `rm75_touch_teleop`.
