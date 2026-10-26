# RM75-6FB kinematics and redundancy design

This package is offline-only: it has no publishers, driver dependencies, or hardware launch file.

## Model and conventions

The build expands the vendor `RM75_6FB/rm_75_6fb_description.urdf.xacro` with
`link7_type:=Link7_6fb`. The kinematic tip remains named `Link7`; the 6FB variant changes
joint 7's origin to `[0, 0, 0.1612]` m and its mesh/inertia. The base is `base_link`, joints
are `joint1` through `joint7` in that order. Pinocchio FK returns `base_link -> Link7`.

Jacobians use `LOCAL_WORLD_ALIGNED` at the Link7 origin. Rows 0..2 are linear velocity in
the base axes and rows 3..5 are angular velocity in the base axes. Finite differences apply
the spatial orientation increment `log(R(q+dq) R(q)^T)` so they test that same convention.

## Adaptive DLS

The primary step is `Jw^T (Jw Jw^T + lambda^2 I)^-1 ew`, with separate translation and
rotation weights. Lambda rises smoothly from `damping_min` as the minimum singular value
falls below its threshold. Every step is bounded, checked against hard joint limits, and
accepted only by backtracking when the weighted task error decreases. The solver reports a
typed status, errors, iterations, timing, damping, singular value and limit margin.

## Redundancy

Joint centering uses the negative gradient of a normalized quadratic and a smooth reciprocal
soft-limit barrier. It is projected with `N = I - J#J`. With a damped inverse this is not an
exact nullspace projector, so `||J N qdot0||` is explicitly measured rather than assumed zero.
The task-space line search remains authoritative; a secondary step cannot be accepted if it
increases the primary weighted pose error.

Two additional optional gradients are supported: `seed-q` minimizes unnecessary total joint
change, and a central finite-difference gradient of `sum(log(sigma_i + 1e-6))` maximizes a
scale-stable manipulability volume. The latter is deliberately optional because fourteen
extra Jacobian evaluations per iteration make it unsuitable for an unmeasured real-time loop.

## CasADi/Ipopt

`scripts/optimization_ik.py` parses the expanded URDF and reconstructs the entire serial
transform chain with CasADi SX operations. Ipopt therefore receives a genuine automatic-
differentiation graph; no numerical Pinocchio function is presented as differentiable. The
NLP has hard joint bounds, position and SO(3)-log orientation costs, seed regularization,
joint-centering regularization, iteration/CPU budgets, dual warm starts and detailed status.
It runs in a pinned, isolated Python environment because the ROS image has no CasADi/Ipopt.
Its symbolic FK was cross-checked against Pinocchio to 1.11e-15 maximum matrix error.
