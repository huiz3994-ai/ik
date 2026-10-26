#!/usr/bin/env python3

import os
import sys
import time
import numpy as np

import rospy
import tf
import moveit_commander

from sensor_msgs.msg import JointState
from rm_msgs.msg import Joint_Speed


MODEL_PATH = (
    "/root/rm75_ws/ppo_deploy/"
    "rm75_ppo_stage8_v4_phase3_300k_numpy.npz"
)

GROUP_NAME = "arm"
BASE_FRAME = "base_link"
EE_LINK = "Link7"

JOINT_ORDER = [
    "joint1",
    "joint2",
    "joint3",
    "joint4",
    "joint5",
    "joint6",
    "joint7",
]

JOINT_LOWER = np.array([
    -3.106,
    -2.2689,
    -3.106,
    -2.356,
    -3.106,
    -2.234,
    -6.280,
], dtype=np.float64)

JOINT_UPPER = np.array([
    3.106,
    2.2689,
    3.106,
    2.356,
    3.106,
    2.234,
    6.280,
], dtype=np.float64)

JOINT_RANGE = JOINT_UPPER - JOINT_LOWER

# MoveIt full velocity limits [rad/s]
MOVEIT_VEL_LIMIT = np.array([
    3.14,
    3.14,
    3.92,
    3.92,
    3.92,
    3.92,
    3.92,
], dtype=np.float64)

# 5% converted to RPM
SAFE_PLAN_RPM = (
    MOVEIT_VEL_LIMIT
    * 0.05
    * 60.0
    / (2.0 * np.pi)
)

# ============================================================
# Conservative application-level DRY-RUN gate limits
# ============================================================

MAX_STATE_AGE = 0.10
MAX_IDLE_SPEED_RPM = 0.20

MAX_CART_STEP = 0.002
MAX_START_ERROR = 0.02
MAX_JOINT_DEVIATION = 0.02

VELOCITY_SCALING = 0.05
ACCELERATION_SCALING = 0.05

ALPHA = 0.7
MAX_DELTA = 0.005


class NumpyActor:
    def __init__(self, path):
        if not os.path.exists(path):
            raise FileNotFoundError(path)

        d = np.load(path)

        self.w0 = d["w0"]
        self.b0 = d["b0"]
        self.w1 = d["w1"]
        self.b1 = d["b1"]
        self.wa = d["wa"]
        self.ba = d["ba"]

        self.low = d["action_low"]
        self.high = d["action_high"]

    def predict(self, obs):
        x = np.tanh(
            self.w0 @ obs + self.b0
        )

        x = np.tanh(
            self.w1 @ x + self.b1
        )

        action = self.wa @ x + self.ba

        return np.clip(
            action,
            self.low,
            self.high,
        )


class SafetyGateDryRun:
    def __init__(self):
        self.actor = NumpyActor(MODEL_PATH)

        self.latest_q = None
        self.latest_joint_time = None

        self.latest_speed = None
        self.latest_speed_time = None

        self.listener = tf.TransformListener()

        rospy.Subscriber(
            "/joint_states",
            JointState,
            self.joint_cb,
            queue_size=1,
        )

        rospy.Subscriber(
            "/rm_driver/Udp_Joint_Speed",
            Joint_Speed,
            self.speed_cb,
            queue_size=20,
        )

        self.group = (
            moveit_commander.MoveGroupCommander(
                GROUP_NAME
            )
        )

        self.group.set_planning_time(5.0)
        self.group.set_num_planning_attempts(5)

        self.group.set_max_velocity_scaling_factor(
            VELOCITY_SCALING
        )

        self.group.set_max_acceleration_scaling_factor(
            ACCELERATION_SCALING
        )

    def joint_cb(self, msg):
        table = dict(
            zip(msg.name, msg.position)
        )

        if not all(
            name in table
            for name in JOINT_ORDER
        ):
            return

        self.latest_q = np.array(
            [table[name] for name in JOINT_ORDER],
            dtype=np.float64,
        )

        self.latest_joint_time = time.monotonic()

    def speed_cb(self, msg):
        speed = np.asarray(
            msg.joint_speed,
            dtype=np.float64,
        )

        if speed.shape != (7,):
            return

        self.latest_speed = speed
        self.latest_speed_time = time.monotonic()

    def normalize_q(self, q):
        return np.clip(
            2.0
            * (q - JOINT_LOWER)
            / JOINT_RANGE
            - 1.0,
            -1.0,
            1.0,
        ).astype(np.float32)

    def wait_for_data(self):
        deadline = time.monotonic() + 5.0

        while not rospy.is_shutdown():
            if (
                self.latest_q is not None
                and self.latest_speed is not None
            ):
                return

            if time.monotonic() > deadline:
                raise RuntimeError(
                    "Timed out waiting for robot state / speed"
                )

            rospy.sleep(0.05)

    def get_ee(self):
        trans, _ = self.listener.lookupTransform(
            BASE_FRAME,
            EE_LINK,
            rospy.Time(0),
        )

        return np.asarray(
            trans,
            dtype=np.float64,
        )

    def unpack_plan(self, result):
        if isinstance(result, tuple):
            return bool(result[0]), result[1]

        traj = result

        return (
            len(traj.joint_trajectory.points) > 0,
            traj,
        )

    def check(self, name, condition, detail):
        status = "PASS ✅" if condition else "FAIL ❌"

        print(
            "{:<34s} {:<8s} {}".format(
                name,
                status,
                detail,
            )
        )

        return bool(condition)

    def run(self):
        self.wait_for_data()
        rospy.sleep(0.3)

        gates = []

        now = time.monotonic()

        q = self.latest_q.copy()
        speed = self.latest_speed.copy()

        joint_age = (
            now - self.latest_joint_time
        )

        speed_age = (
            now - self.latest_speed_time
        )

        print()
        print("=" * 82)
        print("RM75 PPO EXECUTION SAFETY GATE — DRY RUN")
        print("=" * 82)
        print("NO ROBOT EXECUTION IS PERMITTED BY THIS SCRIPT")
        print()

        gates.append(
            self.check(
                "Joint-state freshness",
                joint_age < MAX_STATE_AGE,
                "age={:.4f}s limit={:.2f}s".format(
                    joint_age,
                    MAX_STATE_AGE,
                ),
            )
        )

        gates.append(
            self.check(
                "Speed-state freshness",
                speed_age < MAX_STATE_AGE,
                "age={:.4f}s limit={:.2f}s".format(
                    speed_age,
                    MAX_STATE_AGE,
                ),
            )
        )

        max_idle_speed = float(
            np.max(np.abs(speed))
        )

        gates.append(
            self.check(
                "Robot stationary",
                max_idle_speed < MAX_IDLE_SPEED_RPM,
                "max={:.3f}RPM limit={:.3f}RPM".format(
                    max_idle_speed,
                    MAX_IDLE_SPEED_RPM,
                ),
            )
        )

        ee = self.get_ee()

        # Same validation target: +X 8 cm
        relative_target = np.array(
            [0.08, 0.0, 0.0],
            dtype=np.float32,
        )

        q_norm = self.normalize_q(q)

        prev_action = np.zeros(
            3,
            dtype=np.float32,
        )

        obs = np.concatenate([
            q_norm,
            relative_target,
            prev_action,
        ]).astype(np.float32)

        raw_action = self.actor.predict(obs)

        smooth_action = (
            ALPHA * prev_action
            + (1.0 - ALPHA) * raw_action
        )

        delta_xyz = (
            smooth_action * MAX_DELTA
        )

        cart_step = float(
            np.linalg.norm(delta_xyz)
        )

        gates.append(
            self.check(
                "PPO Cartesian step",
                cart_step <= MAX_CART_STEP,
                "norm={:.3f}mm limit={:.3f}mm".format(
                    cart_step * 1000.0,
                    MAX_CART_STEP * 1000.0,
                ),
            )
        )

        target_xyz = (
            ee + delta_xyz
        )

        pose = (
            self.group
            .get_current_pose(EE_LINK)
            .pose
        )

        pose.position.x = float(target_xyz[0])
        pose.position.y = float(target_xyz[1])
        pose.position.z = float(target_xyz[2])

        self.group.set_start_state_to_current_state()

        self.group.set_pose_target(
            pose,
            EE_LINK,
        )

        result = self.group.plan()

        success, traj = self.unpack_plan(
            result
        )

        self.group.clear_pose_targets()

        points = traj.joint_trajectory.points
        names = list(
            traj.joint_trajectory.joint_names
        )

        gates.append(
            self.check(
                "MoveIt planning",
                success and len(points) > 0,
                "success={} points={}".format(
                    success,
                    len(points),
                ),
            )
        )

        if not success or not points:
            print()
            print("OVERALL GATE: BLOCKED ❌")
            print("No robot command was sent.")
            return

        current_map = dict(
            zip(JOINT_ORDER, q)
        )

        # ========================================================
        # Start state error
        # ========================================================

        p0 = points[0]

        start_errors = []

        for joint_name, value in zip(
            names,
            p0.positions,
        ):
            start_errors.append(
                abs(
                    float(value)
                    - current_map[joint_name]
                )
            )

        max_start_error = max(start_errors)

        gates.append(
            self.check(
                "Plan start-state error",
                max_start_error <= MAX_START_ERROR,
                "max={:.6f}rad ({:.3f}deg)".format(
                    max_start_error,
                    np.degrees(max_start_error),
                ),
            )
        )

        # ========================================================
        # Maximum joint deviation
        # ========================================================

        max_joint_dev = 0.0

        for point in points:
            for joint_name, value in zip(
                names,
                point.positions,
            ):
                dq = abs(
                    float(value)
                    - current_map[joint_name]
                )

                max_joint_dev = max(
                    max_joint_dev,
                    dq,
                )

        gates.append(
            self.check(
                "Plan max joint deviation",
                max_joint_dev <= MAX_JOINT_DEVIATION,
                "max={:.6f}rad ({:.3f}deg)".format(
                    max_joint_dev,
                    np.degrees(max_joint_dev),
                ),
            )
        )

        # ========================================================
        # Independent segment average velocity
        # ========================================================

        max_ratio = 0.0
        worst_joint = None
        worst_rpm = 0.0

        for i in range(1, len(points)):
            p_prev = points[i - 1]
            p_now = points[i]

            dt = (
                p_now.time_from_start.to_sec()
                - p_prev.time_from_start.to_sec()
            )

            if dt <= 0:
                gates.append(
                    self.check(
                        "Trajectory time monotonic",
                        False,
                        "dt={}".format(dt),
                    )
                )
                continue

            for j, joint_name in enumerate(names):
                dq = abs(
                    float(p_now.positions[j])
                    - float(p_prev.positions[j])
                )

                rad_s = dq / dt

                rpm = (
                    rad_s
                    * 60.0
                    / (2.0 * np.pi)
                )

                joint_index = (
                    JOINT_ORDER.index(
                        joint_name
                    )
                )

                ratio = (
                    rpm
                    / SAFE_PLAN_RPM[
                        joint_index
                    ]
                )

                if ratio > max_ratio:
                    max_ratio = ratio
                    worst_joint = joint_name
                    worst_rpm = rpm

        gates.append(
            self.check(
                "Plan segment avg velocity",
                max_ratio <= 1.0,
                "worst={} {:.3f}RPM "
                "usage={:.1f}%".format(
                    worst_joint,
                    worst_rpm,
                    max_ratio * 100.0,
                ),
            )
        )

        print()
        print("-" * 82)

        all_pass = all(gates)

        if all_pass:
            print(
                "OVERALL GATE: PASS ✅ "
                "(DRY RUN ONLY)"
            )
        else:
            print(
                "OVERALL GATE: BLOCKED ❌"
            )

        print()
        print("PPO raw action    :", np.round(raw_action, 4))
        print("PPO smooth action :", np.round(smooth_action, 4))
        print(
            "Delta XYZ [mm]    :",
            np.round(delta_xyz * 1000.0, 3),
        )
        print(
            "Speed feedback RPM:",
            np.round(speed, 3),
        )

        print()
        print(
            "IMPORTANT: this script contains NO execute()/go()."
        )
        print(
            "No robot motion command was sent."
        )
        print("=" * 82)


def main():
    moveit_commander.roscpp_initialize(
        sys.argv
    )

    rospy.init_node(
        "rm75_execution_safety_gate_dryrun",
        anonymous=True,
    )

    try:
        SafetyGateDryRun().run()
    finally:
        moveit_commander.roscpp_shutdown()


if __name__ == "__main__":
    main()
