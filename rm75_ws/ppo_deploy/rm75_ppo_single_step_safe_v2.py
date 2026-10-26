#!/usr/bin/env python3

import argparse
import copy
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

MOVEIT_VEL_LIMIT = np.array([
    3.14,
    3.14,
    3.92,
    3.92,
    3.92,
    3.92,
    3.92,
], dtype=np.float64)

VELOCITY_SCALING = 0.05
ACCELERATION_SCALING = 0.05

SAFE_PLAN_RPM = (
    MOVEIT_VEL_LIMIT
    * VELOCITY_SCALING
    * 60.0
    / (2.0 * np.pi)
)

# ============================================================
# Application-level safety limits
# ============================================================

MAX_STATE_AGE = 0.10
MAX_IDLE_SPEED_RPM = 0.20

MAX_CART_STEP = 0.002          # 2 mm
MAX_START_ERROR = 0.02        # rad
MAX_JOINT_DEVIATION = 0.02    # rad

ALPHA = 0.7
MAX_DELTA = 0.005

# Same validated virtual goal:
# first PPO action toward +X 8 cm.
VIRTUAL_RELATIVE_TARGET = np.array(
    [0.08, 0.0, 0.0],
    dtype=np.float32,
)


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "RM75 PPO single-step safety-gated runner. "
            "Default is DRY RUN."
        )
    )

    parser.add_argument(
        "--execute",
        action="store_true",
        help=(
            "Actually execute exactly ONE validated MoveIt "
            "trajectory. Without this flag, robot cannot move."
        ),
    )

    return parser.parse_args(
        rospy.myargv()[1:]
    )


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

        if self.w0.shape != (64, 13):
            raise RuntimeError(
                "Expected Stage8 13D policy"
            )

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
        ).astype(np.float32)


class SingleStepRunner:
    def __init__(self, execute):
        self.execute_enabled = bool(execute)

        self.actor = NumpyActor(
            MODEL_PATH
        )

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
            zip(
                msg.name,
                msg.position,
            )
        )

        if not all(
            name in table
            for name in JOINT_ORDER
        ):
            return

        self.latest_q = np.array(
            [
                table[name]
                for name in JOINT_ORDER
            ],
            dtype=np.float64,
        )

        self.latest_joint_time = (
            time.monotonic()
        )

    def speed_cb(self, msg):
        speed = np.asarray(
            msg.joint_speed,
            dtype=np.float64,
        )

        if speed.shape != (7,):
            return

        self.latest_speed = speed
        self.latest_speed_time = (
            time.monotonic()
        )

    def wait_for_data(self):
        deadline = (
            time.monotonic() + 5.0
        )

        while not rospy.is_shutdown():
            if (
                self.latest_q is not None
                and self.latest_speed is not None
            ):
                return

            if time.monotonic() > deadline:
                raise RuntimeError(
                    "Timeout waiting for robot state"
                )

            rospy.sleep(0.05)

    def normalize_q(self, q):
        return np.clip(
            2.0
            * (q - JOINT_LOWER)
            / JOINT_RANGE
            - 1.0,
            -1.0,
            1.0,
        ).astype(np.float32)

    def get_ee(self):
        self.listener.waitForTransform(
            BASE_FRAME,
            EE_LINK,
            rospy.Time(0),
            rospy.Duration(2.0),
        )

        trans, _ = (
            self.listener.lookupTransform(
                BASE_FRAME,
                EE_LINK,
                rospy.Time(0),
            )
        )

        return np.asarray(
            trans,
            dtype=np.float64,
        )

    @staticmethod
    def unpack_plan(result):
        if isinstance(result, tuple):
            return (
                bool(result[0]),
                result[1],
            )

        trajectory = result

        return (
            len(
                trajectory
                .joint_trajectory
                .points
            ) > 0,
            trajectory,
        )

    @staticmethod
    def report(name, ok, detail):
        print(
            "{:<34s} {:<8s} {}".format(
                name,
                "PASS ✅" if ok else "FAIL ❌",
                detail,
            )
        )

        return bool(ok)

    def check_fresh_stationary(self):
        now = time.monotonic()

        if (
            self.latest_q is None
            or self.latest_speed is None
        ):
            return False

        joint_age = (
            now - self.latest_joint_time
        )

        speed_age = (
            now - self.latest_speed_time
        )

        max_speed = float(
            np.max(
                np.abs(
                    self.latest_speed
                )
            )
        )

        ok = (
            joint_age < MAX_STATE_AGE
            and speed_age < MAX_STATE_AGE
            and max_speed < MAX_IDLE_SPEED_RPM
        )

        return ok

    def run(self):
        self.wait_for_data()

        rospy.sleep(0.3)

        q = self.latest_q.copy()
        speed = self.latest_speed.copy()

        now = time.monotonic()

        gates = []

        print()
        print("=" * 84)
        print("RM75 PPO SINGLE-STEP SAFETY RUNNER")
        print("=" * 84)

        if self.execute_enabled:
            print(
                "MODE: EXECUTION ARMED ⚠️"
            )
            print(
                "Exactly ONE trajectory may be executed."
            )
        else:
            print(
                "MODE: DRY RUN 🔒"
            )
            print(
                "Execution is locked because --execute "
                "was NOT supplied."
            )

        print("=" * 84)
        print()

        joint_age = (
            now - self.latest_joint_time
        )

        speed_age = (
            now - self.latest_speed_time
        )

        gates.append(
            self.report(
                "Joint-state freshness",
                joint_age < MAX_STATE_AGE,
                "age={:.4f}s".format(
                    joint_age
                ),
            )
        )

        gates.append(
            self.report(
                "Speed-state freshness",
                speed_age < MAX_STATE_AGE,
                "age={:.4f}s".format(
                    speed_age
                ),
            )
        )

        max_idle_speed = float(
            np.max(np.abs(speed))
        )

        gates.append(
            self.report(
                "Robot stationary",
                max_idle_speed
                < MAX_IDLE_SPEED_RPM,
                "max={:.3f}RPM".format(
                    max_idle_speed
                ),
            )
        )

        ee = self.get_ee()

        q_norm = self.normalize_q(q)

        prev_action = np.zeros(
            3,
            dtype=np.float32,
        )

        obs = np.concatenate([
            q_norm,
            VIRTUAL_RELATIVE_TARGET,
            prev_action,
        ]).astype(np.float32)

        raw_action = (
            self.actor.predict(obs)
        )

        if not np.all(
            np.isfinite(raw_action)
        ):
            raise RuntimeError(
                "Non-finite PPO action"
            )

        smooth_action = (
            ALPHA * prev_action
            + (1.0 - ALPHA)
            * raw_action
        ).astype(np.float32)

        delta_xyz = (
            smooth_action
            * MAX_DELTA
        ).astype(np.float64)

        cart_step = float(
            np.linalg.norm(
                delta_xyz
            )
        )

        gates.append(
            self.report(
                "PPO Cartesian step",
                cart_step <= MAX_CART_STEP,
                "norm={:.3f}mm".format(
                    cart_step * 1000.0
                ),
            )
        )

        d_now = float(
            np.linalg.norm(
                VIRTUAL_RELATIVE_TARGET
            )
        )

        d_after = float(
            np.linalg.norm(
                VIRTUAL_RELATIVE_TARGET
                - delta_xyz
            )
        )

        gates.append(
            self.report(
                "PPO local improvement",
                d_after < d_now,
                "{:.3f}mm -> {:.3f}mm".format(
                    d_now * 1000.0,
                    d_after * 1000.0,
                ),
            )
        )

        target_xyz = (
            ee + delta_xyz
        )

        current_pose = (
            self.group
            .get_current_pose(EE_LINK)
            .pose
        )

        target_pose = copy.deepcopy(
            current_pose
        )

        target_pose.position.x = float(
            target_xyz[0]
        )

        target_pose.position.y = float(
            target_xyz[1]
        )

        target_pose.position.z = float(
            target_xyz[2]
        )

        self.group.set_start_state_to_current_state()

        self.group.set_pose_target(
            target_pose,
            EE_LINK,
        )

        plan_result = self.group.plan()

        success, trajectory = (
            self.unpack_plan(
                plan_result
            )
        )

        self.group.clear_pose_targets()

        joint_traj = (
            trajectory.joint_trajectory
        )

        points = list(
            joint_traj.points
        )

        names = list(
            joint_traj.joint_names
        )

        gates.append(
            self.report(
                "MoveIt planning",
                success and len(points) > 0,
                "success={} points={}".format(
                    success,
                    len(points),
                ),
            )
        )

        if (
            not success
            or len(points) == 0
        ):
            print()
            print("OVERALL: BLOCKED ❌")
            print("No robot motion command sent.")
            return

        current_map = dict(
            zip(
                JOINT_ORDER,
                q,
            )
        )

        # ========================================================
        # Plan start-state consistency
        # ========================================================

        start_errors = []

        for joint_name, value in zip(
            names,
            points[0].positions,
        ):
            start_errors.append(
                abs(
                    float(value)
                    - float(
                        current_map[
                            joint_name
                        ]
                    )
                )
            )

        max_start_error = max(
            start_errors
        )

        gates.append(
            self.report(
                "Plan start-state error",
                max_start_error
                <= MAX_START_ERROR,
                "max={:.6f}rad ({:.3f}deg)".format(
                    max_start_error,
                    np.degrees(
                        max_start_error
                    ),
                ),
            )
        )

        # ========================================================
        # Maximum joint excursion
        # ========================================================

        max_joint_deviation = 0.0

        for point in points:
            for joint_name, value in zip(
                names,
                point.positions,
            ):
                dq = abs(
                    float(value)
                    - float(
                        current_map[
                            joint_name
                        ]
                    )
                )

                max_joint_deviation = max(
                    max_joint_deviation,
                    dq,
                )

        gates.append(
            self.report(
                "Plan max joint deviation",
                max_joint_deviation
                <= MAX_JOINT_DEVIATION,
                "max={:.6f}rad ({:.3f}deg)".format(
                    max_joint_deviation,
                    np.degrees(
                        max_joint_deviation
                    ),
                ),
            )
        )

        # ========================================================
        # Segment average velocity audit
        # ========================================================

        max_ratio = 0.0
        worst_joint = None
        worst_rpm = 0.0
        time_valid = True

        for i in range(
            1,
            len(points),
        ):
            t0 = (
                points[i - 1]
                .time_from_start
                .to_sec()
            )

            t1 = (
                points[i]
                .time_from_start
                .to_sec()
            )

            dt = t1 - t0

            if dt <= 0.0:
                time_valid = False
                continue

            for j, joint_name in enumerate(
                names
            ):
                dq = abs(
                    float(
                        points[i]
                        .positions[j]
                    )
                    - float(
                        points[i - 1]
                        .positions[j]
                    )
                )

                rad_s = dq / dt

                rpm = (
                    rad_s
                    * 60.0
                    / (2.0 * np.pi)
                )

                idx = JOINT_ORDER.index(
                    joint_name
                )

                ratio = (
                    rpm
                    / SAFE_PLAN_RPM[idx]
                )

                if ratio > max_ratio:
                    max_ratio = ratio
                    worst_joint = joint_name
                    worst_rpm = rpm

        gates.append(
            self.report(
                "Trajectory time monotonic",
                time_valid,
                "all segment dt > 0",
            )
        )

        gates.append(
            self.report(
                "Plan segment avg velocity",
                (
                    time_valid
                    and max_ratio <= 1.0
                ),
                "worst={} {:.3f}RPM usage={:.1f}%".format(
                    worst_joint,
                    worst_rpm,
                    max_ratio * 100.0,
                ),
            )
        )

        duration = (
            points[-1]
            .time_from_start
            .to_sec()
        )

        gates.append(
            self.report(
                "Trajectory duration",
                duration > 0.0,
                "{:.3f}s".format(
                    duration
                ),
            )
        )

        print()
        print("-" * 84)

        print(
            "PPO raw action     :",
            np.round(
                raw_action,
                4,
            ),
        )

        print(
            "PPO smooth action  :",
            np.round(
                smooth_action,
                4,
            ),
        )

        print(
            "Delta XYZ [mm]     :",
            np.round(
                delta_xyz * 1000.0,
                3,
            ),
        )

        print(
            "Current speed [RPM]:",
            np.round(
                speed,
                3,
            ),
        )

        print(
            "Trajectory duration:",
            "{:.3f}s".format(
                duration
            ),
        )

        all_pass = all(gates)

        print()
        print("=" * 84)

        if not all_pass:
            print(
                "SAFETY GATE: BLOCKED ❌"
            )
            print(
                "Trajectory will NOT execute."
            )
            print("=" * 84)
            return

        print(
            "SAFETY GATE: PASS ✅"
        )

        # ========================================================
        # DEFAULT PATH:
        # ALWAYS STOP HERE.
        # ========================================================

        if not self.execute_enabled:
            print()
            print(
                "DRY RUN LOCK ACTIVE 🔒"
            )
            print(
                "No execute() call will occur."
            )
            print(
                "Robot remains stationary."
            )
            print("=" * 84)
            return

        # ========================================================
        # FINAL PRE-EXECUTION RECHECK
        # ========================================================

        rospy.sleep(0.10)

        if not self.check_fresh_stationary():
            print()
            print(
                "FINAL GATE: BLOCKED ❌"
            )
            print(
                "Robot state became stale or "
                "robot is no longer stationary."
            )
            print("=" * 84)
            return

        q_now = self.latest_q.copy()

        q_now_map = dict(
            zip(
                JOINT_ORDER,
                q_now,
            )
        )

        final_start_error = 0.0

        for joint_name, value in zip(
            names,
            points[0].positions,
        ):
            final_start_error = max(
                final_start_error,
                abs(
                    float(value)
                    - float(
                        q_now_map[
                            joint_name
                        ]
                    )
                ),
            )

        if (
            final_start_error
            > MAX_START_ERROR
        ):
            print()
            print(
                "FINAL GATE: BLOCKED ❌"
            )
            print(
                "Plan start no longer matches robot."
            )
            print(
                "error={:.6f}rad".format(
                    final_start_error
                )
            )
            print("=" * 84)
            return

        print()
        print("=" * 84)
        print(
            "FINAL PRE-EXECUTION GATE: PASS ✅"
        )
        print(
            "Executing exactly ONE trajectory..."
        )
        print("=" * 84)

        # ========================================================
        # THE ONLY MOTION CALL IN THIS SCRIPT
        # ========================================================

        execution_result = (
            self.group.execute(
                trajectory,
                wait=True,
            )
        )

        self.group.stop()

        print()
        print("=" * 84)
        print(
            "Single-step execute() returned:",
            execution_result,
        )
        print(
            "No second PPO step will be generated."
        )
        print(
            "Program exits after this one trajectory."
        )
        print("=" * 84)


def main():
    args = parse_args()

    moveit_commander.roscpp_initialize(
        sys.argv
    )

    rospy.init_node(
        "rm75_ppo_single_step_safe",
        anonymous=True,
    )

    try:
        SingleStepRunner(
            execute=args.execute
        ).run()

    finally:
        moveit_commander.roscpp_shutdown()


if __name__ == "__main__":
    main()
