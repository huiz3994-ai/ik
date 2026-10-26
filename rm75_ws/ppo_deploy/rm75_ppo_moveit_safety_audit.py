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
PLANNING_FRAME = "dummy"
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

JOINT_RANGE = JOINT_UPPER - JOINT_LOWER

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

        if self.w0.shape != (64, 13):
            raise RuntimeError(
                "Expected Stage8 13D policy, got {}".format(
                    self.w0.shape
                )
            )

    def predict(self, obs):
        obs = np.asarray(
            obs,
            dtype=np.float32,
        )

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


class ShadowPlanner:
    def __init__(self):
        self.actor = NumpyActor(
            MODEL_PATH
        )

        self.latest_q = None

        self.listener = (
            tf.TransformListener()
        )

        rospy.Subscriber(
            "/joint_states",
            JointState,
            self.joint_cb,
            queue_size=1,
        )

        print("Initializing MoveIt...")

        self.robot = (
            moveit_commander.RobotCommander()
        )

        self.group = (
            moveit_commander.MoveGroupCommander(
                GROUP_NAME
            )
        )

        self.group.set_planning_time(5.0)
        self.group.set_num_planning_attempts(5)

        # Safety audit only:
        # deliberately slow MoveIt trajectory generation.
        self.group.set_max_velocity_scaling_factor(0.05)
        self.group.set_max_acceleration_scaling_factor(0.05)

        print("Velocity scaling    : 0.05")
        print("Acceleration scaling: 0.05")

        print(
            "Planning frame:",
            self.group.get_planning_frame(),
        )

        print(
            "EE link:",
            self.group.get_end_effector_link(),
        )

    def joint_cb(self, msg):
        name_to_position = dict(
            zip(
                msg.name,
                msg.position,
            )
        )

        if not all(
            name in name_to_position
            for name in JOINT_ORDER
        ):
            return

        self.latest_q = np.array(
            [
                name_to_position[name]
                for name in JOINT_ORDER
            ],
            dtype=np.float32,
        )

    def normalize_q(self, q):
        q_norm = (
            2.0
            * (q - JOINT_LOWER)
            / JOINT_RANGE
            - 1.0
        )

        return np.clip(
            q_norm,
            -1.0,
            1.0,
        ).astype(np.float32)

    def wait_for_state(self):
        rate = rospy.Rate(10)

        while (
            not rospy.is_shutdown()
            and self.latest_q is None
        ):
            rospy.loginfo_throttle(
                2.0,
                "Waiting for /joint_states..."
            )

            rate.sleep()

        if rospy.is_shutdown():
            raise RuntimeError(
                "ROS shutdown before joint state received"
            )

    def get_ee_tf(self):
        self.listener.waitForTransform(
            BASE_FRAME,
            EE_LINK,
            rospy.Time(0),
            rospy.Duration(3.0),
        )

        trans, quat = (
            self.listener.lookupTransform(
                BASE_FRAME,
                EE_LINK,
                rospy.Time(0),
            )
        )

        return (
            np.array(
                trans,
                dtype=np.float32,
            ),
            np.array(
                quat,
                dtype=np.float32,
            ),
        )

    def unpack_plan(self, result):
        """
        Support both MoveIt Python API formats.

        Newer:
          success, trajectory, planning_time, error_code

        Older:
          RobotTrajectory
        """

        if isinstance(result, tuple):
            if len(result) >= 2:
                success = bool(
                    result[0]
                )

                trajectory = result[1]

                planning_time = (
                    float(result[2])
                    if len(result) >= 3
                    else float("nan")
                )

                return (
                    success,
                    trajectory,
                    planning_time,
                )

        trajectory = result

        points = (
            trajectory
            .joint_trajectory
            .points
        )

        return (
            len(points) > 0,
            trajectory,
            float("nan"),
        )

    def run_once(self):
        self.wait_for_state()

        rospy.sleep(0.5)

        q = self.latest_q.copy()

        ee, ee_quat = (
            self.get_ee_tf()
        )

        q_norm = self.normalize_q(
            q
        )

        # ========================================================
        # Same Shadow test:
        # fixed virtual target = current EE + X 8 cm
        # ========================================================

        virtual_target = (
            ee
            + np.array(
                [0.08, 0.0, 0.0],
                dtype=np.float32,
            )
        )

        target_relative = (
            virtual_target - ee
        ).astype(np.float32)

        # First policy step:
        # filter state starts from zero.
        prev_action = np.zeros(
            3,
            dtype=np.float32,
        )

        obs = np.concatenate(
            [
                q_norm,
                target_relative,
                prev_action,
            ]
        ).astype(np.float32)

        raw_action = (
            self.actor.predict(obs)
        )

        smooth_action = (
            ALPHA * prev_action
            + (1.0 - ALPHA)
            * raw_action
        ).astype(np.float32)

        delta_xyz = (
            smooth_action
            * MAX_DELTA
        ).astype(np.float32)

        pose_target_xyz = (
            ee + delta_xyz
        ).astype(np.float32)

        d_now = float(
            np.linalg.norm(
                virtual_target - ee
            )
        )

        d_after_virtual_step = float(
            np.linalg.norm(
                virtual_target
                - pose_target_xyz
            )
        )

        print()
        print("=" * 72)
        print("RM75 Stage 8.6 PPO -> MoveIt SHADOW PLANNING")
        print("=" * 72)
        print("IMPORTANT: PLAN ONLY. NO EXECUTION.")
        print()

        print("Current q:")
        print(
            np.round(q, 6)
        )

        print()
        print("q_norm:")
        print(
            np.round(q_norm, 4)
        )

        print()
        print("Current EE:")
        print(
            np.round(ee, 6)
        )

        print()
        print("Virtual target (+X 8 cm):")
        print(
            np.round(
                virtual_target,
                6,
            )
        )

        print()
        print("PPO raw action:")
        print(
            np.round(
                raw_action,
                4,
            )
        )

        print()
        print("PPO smooth action:")
        print(
            np.round(
                smooth_action,
                4,
            )
        )

        print()
        print("PPO delta XYZ [m]:")
        print(
            np.round(
                delta_xyz,
                6,
            )
        )

        print()
        print(
            "Distance now     : {:.6f} m".format(
                d_now
            )
        )

        print(
            "Distance after Δ : {:.6f} m".format(
                d_after_virtual_step
            )
        )

        print(
            "PPO local improve:",
            d_after_virtual_step < d_now,
        )

        # ========================================================
        # Construct MoveIt target pose.
        #
        # dummy == base_link exactly, verified by TF.
        #
        # Keep current orientation unchanged.
        # ========================================================

        current_pose = (
            self.group.get_current_pose(
                EE_LINK
            ).pose
        )

        target_pose = current_pose

        target_pose.position.x = float(
            pose_target_xyz[0]
        )

        target_pose.position.y = float(
            pose_target_xyz[1]
        )

        target_pose.position.z = float(
            pose_target_xyz[2]
        )

        # Keep orientation exactly as current MoveIt pose.
        # No orientation command from PPO.

        self.group.set_start_state_to_current_state()

        self.group.set_pose_target(
            target_pose,
            EE_LINK,
        )

        print()
        print("=" * 72)
        print("Calling MoveIt.plan()")
        print("NO execute(), NO go()")
        print("=" * 72)

        result = self.group.plan()

        (
            success,
            trajectory,
            planning_time,
        ) = self.unpack_plan(
            result
        )

        self.group.clear_pose_targets()

        print()
        print("=" * 72)
        print("MoveIt Planning Result")
        print("=" * 72)

        print(
            "Planning success:",
            success,
        )

        print(
            "Planning time   :",
            planning_time,
        )

        joint_traj = (
            trajectory.joint_trajectory
        )

        print(
            "Trajectory joints:",
            list(
                joint_traj.joint_names
            ),
        )

        print(
            "Trajectory points:",
            len(joint_traj.points),
        )

        if (
            not success
            or len(joint_traj.points) == 0
        ):
            print()
            print("PLAN FAILED ❌")
            print("No command was executed.")
            return

        # ========================================================
        # Analyze planned joint trajectory
        # ========================================================

        traj_names = list(
            joint_traj.joint_names
        )

        name_to_current_q = {
            name: float(value)
            for name, value
            in zip(
                JOINT_ORDER,
                q,
            )
        }

        final_point = (
            joint_traj.points[-1]
        )

        final_delta = []

        for name, value in zip(
            traj_names,
            final_point.positions,
        ):
            if name in name_to_current_q:
                final_delta.append(
                    float(value)
                    - name_to_current_q[name]
                )

        final_delta = np.array(
            final_delta,
            dtype=np.float64,
        )

        max_abs_deviation = 0.0

        for point in joint_traj.points:
            for name, value in zip(
                traj_names,
                point.positions,
            ):
                if name not in name_to_current_q:
                    continue

                deviation = abs(
                    float(value)
                    - name_to_current_q[name]
                )

                max_abs_deviation = max(
                    max_abs_deviation,
                    deviation,
                )

        duration = (
            final_point.time_from_start.to_sec()
        )

        # ========================================================
        # Dynamic trajectory audit
        # ========================================================

        max_velocity = 0.0
        max_acceleration = 0.0

        # Independent segment-speed audit.
        # Important because a 2-point MoveIt trajectory can report
        # zero velocity at both endpoints even though motion occurs.
        max_segment_velocity = 0.0
        max_segment_velocity_joint = None

        segment_velocity_records = []

        for i in range(1, len(joint_traj.points)):
            p0 = joint_traj.points[i - 1]
            p1 = joint_traj.points[i]

            t0 = p0.time_from_start.to_sec()
            t1 = p1.time_from_start.to_sec()

            dt = t1 - t0

            if dt <= 0.0:
                continue

            for j, joint_name in enumerate(joint_traj.joint_names):
                dq = (
                    float(p1.positions[j])
                    - float(p0.positions[j])
                )

                avg_v = abs(dq) / dt

                segment_velocity_records.append(
                    (
                        i - 1,
                        i,
                        joint_name,
                        dq,
                        dt,
                        avg_v,
                    )
                )

                if avg_v > max_segment_velocity:
                    max_segment_velocity = avg_v
                    max_segment_velocity_joint = joint_name

        velocity_data_found = False
        acceleration_data_found = False

        for point in joint_traj.points:
            if len(point.velocities) > 0:
                velocity_data_found = True

                max_velocity = max(
                    max_velocity,
                    max(
                        abs(float(v))
                        for v in point.velocities
                    ),
                )

            if len(point.accelerations) > 0:
                acceleration_data_found = True

                max_acceleration = max(
                    max_acceleration,
                    max(
                        abs(float(a))
                        for a in point.accelerations
                    ),
                )

        print()
        print(
            "Final joint delta [rad]:"
        )

        print(
            np.round(
                final_delta,
                6,
            )
        )

        print(
            "Final joint delta [deg]:"
        )

        print(
            np.round(
                np.degrees(
                    final_delta
                ),
                4,
            )
        )

        print()
        print(
            "Max joint deviation:"
        )

        print(
            "{:.6f} rad = {:.3f} deg".format(
                max_abs_deviation,
                np.degrees(
                    max_abs_deviation
                ),
            )
        )

        print(
            "Trajectory duration:",
            "{:.3f} s".format(
                duration
            )
        )

        print()

        if velocity_data_found:
            print(
                "Max trajectory velocity:",
                "{:.6f} rad/s = {:.3f} deg/s".format(
                    max_velocity,
                    np.degrees(max_velocity),
                )
            )
        else:
            print(
                "Max trajectory velocity: "
                "NO VELOCITY DATA"
            )

        if acceleration_data_found:
            print(
                "Max trajectory acceleration:",
                "{:.6f} rad/s^2 = {:.3f} deg/s^2".format(
                    max_acceleration,
                    np.degrees(max_acceleration),
                )
            )
        else:
            print(
                "Max trajectory acceleration: "
                "NO ACCELERATION DATA"
            )

        print()

        print("=" * 72)
        print("Independent Segment Velocity Audit")
        print("=" * 72)

        for (
            seg0,
            seg1,
            joint_name,
            dq,
            dt,
            avg_v,
        ) in segment_velocity_records:
            print(
                "{} seg {}->{} | "
                "dq={:+.6f} rad | "
                "dt={:.6f} s | "
                "avg_v={:.6f} rad/s = {:.3f} deg/s".format(
                    joint_name,
                    seg0,
                    seg1,
                    dq,
                    dt,
                    avg_v,
                    np.degrees(avg_v),
                )
            )

        print()

        print(
            "Max segment avg velocity:",
            "{:.6f} rad/s = {:.3f} deg/s | joint={}".format(
                max_segment_velocity,
                np.degrees(max_segment_velocity),
                max_segment_velocity_joint,
            )
        )

        print()

        print("=" * 72)
        print("Raw Trajectory Point Data")
        print("=" * 72)

        for i, point in enumerate(joint_traj.points):
            print()
            print(
                "Point {} | t={:.6f} s".format(
                    i,
                    point.time_from_start.to_sec(),
                )
            )

            print(
                "  positions     =",
                np.round(
                    np.asarray(point.positions),
                    6,
                ),
            )

            print(
                "  velocities    =",
                np.round(
                    np.asarray(point.velocities),
                    6,
                )
                if len(point.velocities)
                else "[]",
            )

            print(
                "  accelerations =",
                np.round(
                    np.asarray(point.accelerations),
                    6,
                )
                if len(point.accelerations)
                else "[]",
            )

        print()
        print("=" * 72)
        print("SHADOW PLAN COMPLETE ✅")
        print("NO ROBOT MOTION COMMAND WAS SENT.")
        print("=" * 72)


def main():
    moveit_commander.roscpp_initialize(
        sys.argv
    )

    rospy.init_node(
        "rm75_ppo_moveit_shadow",
        anonymous=True,
    )

    try:
        node = ShadowPlanner()

        node.run_once()

    finally:
        moveit_commander.roscpp_shutdown()


if __name__ == "__main__":
    main()
