#!/usr/bin/env python3

import os
import numpy as np
import rospy
import tf

from sensor_msgs.msg import JointState


MODEL_PATH = (
    "/root/rm75_ws/ppo_deploy/"
    "rm75_ppo_stage8_v4_phase3_300k_numpy.npz"
)

BASE_FRAME = "base_link"
EE_FRAME = "Link7"

JOINT_ORDER = [
    "joint1",
    "joint2",
    "joint3",
    "joint4",
    "joint5",
    "joint6",
    "joint7",
]

JOINT_LOWER = np.array(
    [
        -3.106,
        -2.2689,
        -3.106,
        -2.356,
        -3.106,
        -2.234,
        -6.280,
    ],
    dtype=np.float32,
)

JOINT_UPPER = np.array(
    [
        3.106,
        2.2689,
        3.106,
        2.356,
        3.106,
        2.234,
        6.280,
    ],
    dtype=np.float32,
)

JOINT_RANGE = (
    JOINT_UPPER - JOINT_LOWER
)

ALPHA = 0.7
MAX_DELTA = 0.005

# 仍然只是仿真阶段使用的 placeholder workspace。
# Shadow Mode 中只用于显示/检查，不会发送任何运动命令。
WORKSPACE_MIN = np.array(
    [-0.30, -0.40, 0.20],
    dtype=np.float32,
)

WORKSPACE_MAX = np.array(
    [0.60, 0.40, 0.90],
    dtype=np.float32,
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
                "Expected 13D Stage8 policy, got w0={}".format(
                    self.w0.shape
                )
            )

    def predict(self, obs):
        obs = np.asarray(
            obs,
            dtype=np.float32,
        )

        if obs.shape != (13,):
            raise RuntimeError(
                "Expected obs (13,), got {}".format(
                    obs.shape
                )
            )

        x = np.tanh(
            self.w0 @ obs + self.b0
        )

        x = np.tanh(
            self.w1 @ x + self.b1
        )

        action = (
            self.wa @ x + self.ba
        )

        return np.clip(
            action,
            self.low,
            self.high,
        ).astype(np.float32)


class ShadowNode:
    def __init__(self):
        self.actor = NumpyActor(
            MODEL_PATH
        )

        self.latest_q = None

        self.prev_action = np.zeros(
            3,
            dtype=np.float32,
        )

        self.fixed_target = None

        self.listener = (
            tf.TransformListener()
        )

        self.sub = rospy.Subscriber(
            "/joint_states",
            JointState,
            self.joint_cb,
            queue_size=1,
        )

    def joint_cb(self, msg):
        name_to_pos = dict(
            zip(msg.name, msg.position)
        )

        if not all(
            name in name_to_pos
            for name in JOINT_ORDER
        ):
            return

        self.latest_q = np.array(
            [
                name_to_pos[name]
                for name in JOINT_ORDER
            ],
            dtype=np.float32,
        )

    def normalize_q(self, q):
        q_norm = (
            2.0
            * (
                q - JOINT_LOWER
            )
            / JOINT_RANGE
            - 1.0
        )

        return np.clip(
            q_norm,
            -1.0,
            1.0,
        ).astype(np.float32)

    def get_ee(self):
        try:
            trans, _ = (
                self.listener.lookupTransform(
                    BASE_FRAME,
                    EE_FRAME,
                    rospy.Time(0),
                )
            )

            return np.asarray(
                trans,
                dtype=np.float32,
            )

        except (
            tf.LookupException,
            tf.ConnectivityException,
            tf.ExtrapolationException,
        ):
            return None

    def run(self):
        rate = rospy.Rate(10)

        count = 0

        while (
            not rospy.is_shutdown()
            and count < 50
        ):
            if self.latest_q is None:
                rospy.loginfo_throttle(
                    2.0,
                    "Waiting for /joint_states..."
                )
                rate.sleep()
                continue

            ee = self.get_ee()

            if ee is None:
                rospy.loginfo_throttle(
                    2.0,
                    "Waiting for TF base_link -> Link7..."
                )
                rate.sleep()
                continue

            # 第一次拿到真机 EE 后：
            # 创建固定 +X 8 cm 虚拟目标。
            if self.fixed_target is None:
                self.fixed_target = (
                    ee
                    + np.array(
                        [0.08, 0.0, 0.0],
                        dtype=np.float32,
                    )
                )

                print()
                print("=" * 72)
                print("Stage 8 v4 REAL ROBOT SHADOW MODE")
                print("=" * 72)
                print("READ ONLY: NO MOTION COMMANDS")
                print()
                print("Initial EE :", ee)
                print(
                    "Target     :",
                    self.fixed_target,
                )
                print(
                    "Target delta:",
                    self.fixed_target - ee,
                )
                print("=" * 72)

            q = self.latest_q.copy()

            q_norm = self.normalize_q(q)

            target_relative = (
                self.fixed_target - ee
            ).astype(np.float32)

            # ====================================================
            # Stage8 observation:
            #
            # q_norm          7
            # target_relative 3
            # prev_action     3
            # = 13
            # ====================================================

            obs = np.concatenate(
                [
                    q_norm,
                    target_relative,
                    self.prev_action,
                ]
            ).astype(np.float32)

            raw_action = (
                self.actor.predict(obs)
            )

            smooth_action = (
                ALPHA
                * self.prev_action
                + (1.0 - ALPHA)
                * raw_action
            ).astype(np.float32)

            self.prev_action = (
                smooth_action.copy()
            )

            delta_xyz = (
                smooth_action
                * MAX_DELTA
            ).astype(np.float32)

            # 这里只是假想 command，绝不发布。
            cmd_xyz_raw = (
                ee + delta_xyz
            ).astype(np.float32)

            cmd_xyz = np.clip(
                cmd_xyz_raw,
                WORKSPACE_MIN,
                WORKSPACE_MAX,
            ).astype(np.float32)

            workspace_hit = bool(
                not np.allclose(
                    cmd_xyz_raw,
                    cmd_xyz,
                    atol=1e-8,
                )
            )

            d_now = float(
                np.linalg.norm(
                    self.fixed_target - ee
                )
            )

            # 假设真正走了这一小步，
            # 距离会变成多少？
            d_cmd = float(
                np.linalg.norm(
                    self.fixed_target
                    - cmd_xyz
                )
            )

            improves = bool(
                d_cmd < d_now
            )

            print()
            print(
                "[{:02d}/50]".format(
                    count + 1
                )
            )

            print(
                "q_norm :",
                np.round(q_norm, 4),
            )

            print(
                "Rel XYZ:",
                np.round(
                    target_relative,
                    5,
                ),
            )

            print(
                "Raw    :",
                np.round(
                    raw_action,
                    4,
                ),
            )

            print(
                "Smooth :",
                np.round(
                    smooth_action,
                    4,
                ),
            )

            print(
                "Delta m:",
                np.round(
                    delta_xyz,
                    6,
                ),
            )

            print(
                "d_now  : {:.6f} m".format(
                    d_now
                )
            )

            print(
                "d_cmd  : {:.6f} m".format(
                    d_cmd
                )
            )

            print(
                "Improve:",
                improves,
                "| workspace_hit:",
                workspace_hit,
            )

            count += 1

            rate.sleep()

        print()
        print("=" * 72)
        print("Shadow Mode finished.")
        print("NO robot command was published.")
        print("=" * 72)


def main():
    rospy.init_node(
        "rm75_ppo_shadow_stage8",
        anonymous=True,
    )

    node = ShadowNode()

    node.run()


if __name__ == "__main__":
    main()
