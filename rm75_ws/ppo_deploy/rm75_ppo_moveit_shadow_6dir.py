#!/usr/bin/env python3

import os
import sys
import numpy as np
import rospy
import tf
import moveit_commander

from sensor_msgs.msg import JointState


MODEL_PATH = (
    "/root/rm75_ws/ppo_deploy/"
    "rm75_ppo_stage8_v4_phase3_300k_numpy.npz"
)

GROUP_NAME = "arm"
BASE_FRAME = "base_link"
EE_LINK = "Link7"

JOINT_ORDER = [
    "joint1", "joint2", "joint3", "joint4",
    "joint5", "joint6", "joint7",
]

JOINT_LOWER = np.array([
    -3.106, -2.2689, -3.106, -2.356,
    -3.106, -2.234, -6.280,
], dtype=np.float32)

JOINT_UPPER = np.array([
    3.106, 2.2689, 3.106, 2.356,
    3.106, 2.234, 6.280,
], dtype=np.float32)

JOINT_RANGE = JOINT_UPPER - JOINT_LOWER

ALPHA = 0.7
MAX_DELTA = 0.005

DIRECTIONS = [
    ("+X", np.array([ 0.08,  0.00,  0.00], dtype=np.float32)),
    ("-X", np.array([-0.08,  0.00,  0.00], dtype=np.float32)),
    ("+Y", np.array([ 0.00,  0.08,  0.00], dtype=np.float32)),
    ("-Y", np.array([ 0.00, -0.08,  0.00], dtype=np.float32)),
    ("+Z", np.array([ 0.00,  0.00,  0.08], dtype=np.float32)),
    ("-Z", np.array([ 0.00,  0.00, -0.08], dtype=np.float32)),
]


class NumpyActor:
    def __init__(self, path):
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
        ).astype(np.float32)


class Tester:
    def __init__(self):
        self.actor = NumpyActor(MODEL_PATH)

        self.latest_q = None

        self.listener = tf.TransformListener()

        rospy.Subscriber(
            "/joint_states",
            JointState,
            self.cb,
            queue_size=1,
        )

        self.group = (
            moveit_commander.MoveGroupCommander(
                GROUP_NAME
            )
        )

        self.group.set_planning_time(5.0)
        self.group.set_num_planning_attempts(5)

    def cb(self, msg):
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
            dtype=np.float32,
        )

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
        trans, _ = (
            self.listener.lookupTransform(
                BASE_FRAME,
                EE_LINK,
                rospy.Time(0),
            )
        )

        return np.asarray(
            trans,
            dtype=np.float32,
        )

    def unpack_plan(self, result):
        if isinstance(result, tuple):
            return (
                bool(result[0]),
                result[1],
            )

        trajectory = result

        success = (
            len(
                trajectory
                .joint_trajectory
                .points
            ) > 0
        )

        return success, trajectory

    def run(self):
        while (
            not rospy.is_shutdown()
            and self.latest_q is None
        ):
            rospy.sleep(0.1)

        rospy.sleep(0.5)

        print("=" * 78)
        print("Stage 8.7 Six-Direction MoveIt Shadow Planning")
        print("PLAN ONLY — NO ROBOT EXECUTION")
        print("=" * 78)

        results = []

        for name, target_offset in DIRECTIONS:
            q = self.latest_q.copy()
            q_norm = self.normalize_q(q)

            ee = self.get_ee()

            target_relative = (
                target_offset.copy()
            )

            prev_action = np.zeros(
                3,
                dtype=np.float32,
            )

            obs = np.concatenate([
                q_norm,
                target_relative,
                prev_action,
            ]).astype(np.float32)

            raw = self.actor.predict(obs)

            smooth = (
                ALPHA * prev_action
                + (1.0 - ALPHA) * raw
            ).astype(np.float32)

            delta = (
                smooth * MAX_DELTA
            ).astype(np.float32)

            target_xyz = (
                ee + delta
            ).astype(np.float32)

            d_now = float(
                np.linalg.norm(
                    target_relative
                )
            )

            d_cmd = float(
                np.linalg.norm(
                    target_offset - delta
                )
            )

            improve = (
                d_cmd < d_now
            )

            current_pose = (
                self.group
                .get_current_pose(EE_LINK)
                .pose
            )

            current_pose.position.x = float(
                target_xyz[0]
            )
            current_pose.position.y = float(
                target_xyz[1]
            )
            current_pose.position.z = float(
                target_xyz[2]
            )

            self.group.set_start_state_to_current_state()

            self.group.set_pose_target(
                current_pose,
                EE_LINK,
            )

            plan_result = self.group.plan()

            success, trajectory = (
                self.unpack_plan(
                    plan_result
                )
            )

            self.group.clear_pose_targets()

            points = (
                trajectory
                .joint_trajectory
                .points
            )

            max_joint_delta = float("nan")

            if success and points:
                traj_names = list(
                    trajectory
                    .joint_trajectory
                    .joint_names
                )

                q_table = dict(
                    zip(
                        JOINT_ORDER,
                        q,
                    )
                )

                max_joint_delta = 0.0

                for point in points:
                    for joint_name, value in zip(
                        traj_names,
                        point.positions,
                    ):
                        if joint_name not in q_table:
                            continue

                        dq = abs(
                            float(value)
                            - float(
                                q_table[
                                    joint_name
                                ]
                            )
                        )

                        max_joint_delta = max(
                            max_joint_delta,
                            dq,
                        )

            results.append(
                (
                    name,
                    improve,
                    success,
                    d_cmd,
                    max_joint_delta,
                )
            )

            print()
            print("-" * 78)
            print("Direction:", name)
            print(
                "Target relative:",
                target_relative,
            )
            print(
                "Raw action     :",
                np.round(raw, 4),
            )
            print(
                "Smooth action  :",
                np.round(smooth, 4),
            )
            print(
                "Delta XYZ [mm] :",
                np.round(
                    delta * 1000.0,
                    3,
                ),
            )
            print(
                "d_now          : {:.6f} m".format(
                    d_now
                )
            )
            print(
                "d_cmd          : {:.6f} m".format(
                    d_cmd
                )
            )
            print(
                "Local improve  :",
                improve,
            )
            print(
                "MoveIt plan    :",
                success,
            )

            if success:
                print(
                    "Max joint Δ    : "
                    "{:.4f} deg".format(
                        np.degrees(
                            max_joint_delta
                        )
                    )
                )

        print()
        print("=" * 78)
        print("SUMMARY")
        print("=" * 78)

        all_pass = True

        for (
            name,
            improve,
            success,
            d_cmd,
            max_dq,
        ) in results:
            passed = (
                improve and success
            )

            all_pass &= passed

            print(
                "{:>2s}: improve={} "
                "plan={} "
                "d_cmd={:.4f}m "
                "max_joint={:.3f}deg".format(
                    name,
                    improve,
                    success,
                    d_cmd,
                    np.degrees(max_dq)
                    if np.isfinite(max_dq)
                    else float("nan"),
                )
            )

        print()
        print(
            "Six-direction result:",
            "PASS ✅"
            if all_pass
            else "CHECK REQUIRED ⚠️"
        )

        print()
        print(
            "NO execute()/go() was called."
        )


def main():
    moveit_commander.roscpp_initialize(
        sys.argv
    )

    rospy.init_node(
        "rm75_ppo_moveit_shadow_6dir",
        anonymous=True,
    )

    try:
        Tester().run()
    finally:
        moveit_commander.roscpp_shutdown()


if __name__ == "__main__":
    main()
