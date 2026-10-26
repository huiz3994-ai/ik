#!/usr/bin/env python3

import os
import numpy as np

import rospy
import tf

from sensor_msgs.msg import JointState


# ============================================================
# PPO / Safety Layer parameters
# 必须和 Stage 7 v2 保持一致
# ============================================================

ALPHA = 0.7
MAX_DELTA = 0.005

JOINT_ORDER = [
    "joint1",
    "joint2",
    "joint3",
    "joint4",
    "joint5",
    "joint6",
    "joint7",
]

BASE_FRAME = "base_link"
EE_FRAME = "Link7"

# 注意：
# 这仍然只是 PyBullet 阶段使用的 workspace。
# Shadow Mode 中只用于计算/报警，不执行真机运动。
WORKSPACE_MIN = np.array(
    [-0.30, -0.40, 0.20],
    dtype=np.float32,
)

WORKSPACE_MAX = np.array(
    [0.60, 0.40, 0.90],
    dtype=np.float32,
)


class NumpyPPOPolicy:
    def __init__(self, model_path):
        data = np.load(model_path)

        self.w0 = data["w0"].astype(np.float32)
        self.b0 = data["b0"].astype(np.float32)

        self.w1 = data["w1"].astype(np.float32)
        self.b1 = data["b1"].astype(np.float32)

        self.wa = data["wa"].astype(np.float32)
        self.ba = data["ba"].astype(np.float32)

        self.action_low = data["action_low"].astype(np.float32)
        self.action_high = data["action_high"].astype(np.float32)

        obs_dim = int(data["observation_dim"][0])
        act_dim = int(data["action_dim"][0])

        if obs_dim != 16:
            raise RuntimeError(
                "Expected observation dim 16, got {}".format(obs_dim)
            )

        if act_dim != 3:
            raise RuntimeError(
                "Expected action dim 3, got {}".format(act_dim)
            )

    def predict(self, obs):
        obs = np.asarray(obs, dtype=np.float32)

        if obs.shape != (16,):
            raise RuntimeError(
                "Expected obs shape (16,), got {}".format(obs.shape)
            )

        x = np.tanh(
            self.w0.dot(obs) + self.b0
        )

        x = np.tanh(
            self.w1.dot(x) + self.b1
        )

        action = (
            self.wa.dot(x) + self.ba
        )

        action = np.clip(
            action,
            self.action_low,
            self.action_high,
        )

        return action.astype(np.float32)


class RM75PPOShadow:
    def __init__(self):
        rospy.init_node(
            "rm75_ppo_shadow",
            anonymous=False,
        )

        print("=" * 70)
        print("RM75 PPO SHADOW MODE")
        print("=" * 70)
        print("READ ONLY")
        print("NO robot motion command will be published.")
        print("=" * 70)

        # --------------------------------------------------------
        # Load policy
        # --------------------------------------------------------

        default_model = os.path.join(
            os.path.dirname(os.path.abspath(__file__)),
            "rm75_ppo_stage7_v2_200k_numpy.npz",
        )

        model_path = rospy.get_param(
            "~model_path",
            default_model,
        )

        self.policy = NumpyPPOPolicy(
            model_path,
        )

        rospy.loginfo(
            "Loaded NumPy PPO: %s",
            model_path,
        )

        # --------------------------------------------------------
        # Filter state
        # Stage 7 v2 Observation 的最后 3 维
        # --------------------------------------------------------

        self.prev_action = np.zeros(
            3,
            dtype=np.float32,
        )

        # --------------------------------------------------------
        # Real joint state
        # --------------------------------------------------------

        self.joint_positions = None

        self.joint_sub = rospy.Subscriber(
            "/joint_states",
            JointState,
            self.joint_callback,
            queue_size=1,
        )

        # --------------------------------------------------------
        # TF
        # --------------------------------------------------------

        self.tf_listener = tf.TransformListener()

        # --------------------------------------------------------
        # Shadow target
        #
        # 第一版不用视觉目标。
        # 启动时以当前真机 EE 为参考，
        # 默认在 base +X 方向放一个 8 cm 的虚拟目标。
        #
        # 8 cm 在训练时 5~15 cm 目标距离范围内。
        # --------------------------------------------------------

        self.target_dx = float(
            rospy.get_param("~target_dx", 0.08)
        )

        self.target_dy = float(
            rospy.get_param("~target_dy", 0.0)
        )

        self.target_dz = float(
            rospy.get_param("~target_dz", 0.0)
        )

        self.target_position = None

        target_distance = np.linalg.norm(
            [
                self.target_dx,
                self.target_dy,
                self.target_dz,
            ]
        )

        if not 0.05 <= target_distance <= 0.15:
            raise RuntimeError(
                "Shadow target offset must be within "
                "the training range 0.05~0.15 m. "
                "Current distance = {:.4f} m".format(
                    target_distance
                )
            )

        self.rate = rospy.Rate(10)

    def joint_callback(self, msg):
        if len(msg.name) != len(msg.position):
            return

        mapping = {
            name: position
            for name, position in zip(
                msg.name,
                msg.position,
            )
        }

        missing = [
            name
            for name in JOINT_ORDER
            if name not in mapping
        ]

        if missing:
            rospy.logwarn_throttle(
                2.0,
                "Missing joints: {}".format(missing),
            )
            return

        self.joint_positions = np.array(
            [
                mapping[name]
                for name in JOINT_ORDER
            ],
            dtype=np.float32,
        )

    def get_ee_position(self):
        try:
            translation, rotation = (
                self.tf_listener.lookupTransform(
                    BASE_FRAME,
                    EE_FRAME,
                    rospy.Time(0),
                )
            )

        except (
            tf.LookupException,
            tf.ConnectivityException,
            tf.ExtrapolationException,
        ) as exc:

            rospy.logwarn_throttle(
                2.0,
                "TF unavailable: {}".format(exc),
            )

            return None

        return np.array(
            translation,
            dtype=np.float32,
        )

    def build_observation(
        self,
        q,
        ee_position,
    ):
        target_relative = (
            self.target_position
            - ee_position
        )

        obs = np.concatenate(
            [
                q,
                ee_position,
                target_relative,
                self.prev_action,
            ]
        ).astype(np.float32)

        if obs.shape != (16,):
            raise RuntimeError(
                "Observation shape error: {}".format(
                    obs.shape
                )
            )

        if not np.all(np.isfinite(obs)):
            raise RuntimeError(
                "Observation contains NaN/Inf"
            )

        return obs

    def run(self):

        rospy.loginfo(
            "Waiting for /joint_states and TF %s -> %s ...",
            BASE_FRAME,
            EE_FRAME,
        )

        while not rospy.is_shutdown():

            if self.joint_positions is None:
                self.rate.sleep()
                continue

            ee_position = self.get_ee_position()

            if ee_position is None:
                self.rate.sleep()
                continue

            # ----------------------------------------------------
            # Lock virtual target once
            # ----------------------------------------------------

            if self.target_position is None:
                self.target_position = (
                    ee_position
                    + np.array(
                        [
                            self.target_dx,
                            self.target_dy,
                            self.target_dz,
                        ],
                        dtype=np.float32,
                    )
                )

                rospy.loginfo(
                    "Initial real EE: [%.4f, %.4f, %.4f]",
                    ee_position[0],
                    ee_position[1],
                    ee_position[2],
                )

                rospy.loginfo(
                    "Shadow target : [%.4f, %.4f, %.4f]",
                    self.target_position[0],
                    self.target_position[1],
                    self.target_position[2],
                )

            q = self.joint_positions.copy()

            # ----------------------------------------------------
            # 16D Observation
            # ----------------------------------------------------

            obs = self.build_observation(
                q,
                ee_position,
            )

            # ----------------------------------------------------
            # PPO deterministic inference
            # ----------------------------------------------------

            raw_action = self.policy.predict(
                obs
            )

            # ----------------------------------------------------
            # Same Stage 7 Safety Layer
            # ----------------------------------------------------

            raw_action = np.clip(
                raw_action,
                -1.0,
                1.0,
            )

            smooth_action = (
                ALPHA * self.prev_action
                + (1.0 - ALPHA) * raw_action
            )

            delta_xyz = (
                smooth_action * MAX_DELTA
            )

            shadow_target_ee = (
                ee_position + delta_xyz
            )

            workspace_clipped = np.clip(
                shadow_target_ee,
                WORKSPACE_MIN,
                WORKSPACE_MAX,
            )

            hit_workspace_limit = not np.allclose(
                shadow_target_ee,
                workspace_clipped,
                atol=1e-8,
            )

            distance = float(
                np.linalg.norm(
                    self.target_position
                    - ee_position
                )
            )

            # IMPORTANT:
            # Observation 下一时刻需要看到这一状态
            self.prev_action = (
                smooth_action.copy()
            )

            # ----------------------------------------------------
            # READ ONLY LOG
            #
            # 没有 Publisher
            # 没有 MoveIt execute
            # 没有 rm_driver command
            # ----------------------------------------------------

            msg = (
                "\n"
                "========== PPO SHADOW ==========\n"
                "q       : {}\n"
                "EE      : {}\n"
                "Target  : {}\n"
                "Distance: {:.4f} m\n"
                "Raw     : {}\n"
                "Smooth  : {}\n"
                "Delta   : {} m\n"
                "Cmd XYZ : {}\n"
                "WS clip : {}\n"
                "WS hit  : {}\n"
                "================================"
            ).format(
                np.array2string(
                    q,
                    precision=3,
                    suppress_small=True,
                ),
                np.array2string(
                    ee_position,
                    precision=4,
                    suppress_small=True,
                ),
                np.array2string(
                    self.target_position,
                    precision=4,
                    suppress_small=True,
                ),
                distance,
                np.array2string(
                    raw_action,
                    precision=4,
                    suppress_small=True,
                ),
                np.array2string(
                    smooth_action,
                    precision=4,
                    suppress_small=True,
                ),
                np.array2string(
                    delta_xyz,
                    precision=5,
                    suppress_small=True,
                ),
                np.array2string(
                    shadow_target_ee,
                    precision=4,
                    suppress_small=True,
                ),
                np.array2string(
                    workspace_clipped,
                    precision=4,
                    suppress_small=True,
                ),
                hit_workspace_limit,
            )

            rospy.loginfo_throttle(
                1.0,
                msg,
            )

            self.rate.sleep()


if __name__ == "__main__":
    node = RM75PPOShadow()
    node.run()
