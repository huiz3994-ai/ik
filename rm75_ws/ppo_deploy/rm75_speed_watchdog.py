#!/usr/bin/env python3

import numpy as np
import rospy

from rm_msgs.msg import Joint_Speed


TOPIC = "/rm_driver/Udp_Joint_Speed"

# MoveIt 5% velocity limits converted to RPM.
#
# joint1/2:
# 3.14 rad/s * 0.05 ≈ 0.157 rad/s ≈ 1.50 RPM
#
# joint3~7:
# 3.92 rad/s * 0.05 ≈ 0.196 rad/s ≈ 1.87 RPM
RPM_LIMITS = np.array(
    [
        1.50,
        1.50,
        1.87,
        1.87,
        1.87,
        1.87,
        1.87,
    ],
    dtype=np.float64,
)

count = 0
max_seen = np.zeros(7, dtype=np.float64)


def cb(msg):
    global count
    global max_seen

    speeds = np.asarray(
        msg.joint_speed,
        dtype=np.float64,
    )

    if speeds.shape != (7,):
        rospy.logerr(
            "Expected 7 joint speeds, got %s",
            speeds.shape,
        )
        return

    abs_speed = np.abs(speeds)

    max_seen = np.maximum(
        max_seen,
        abs_speed,
    )

    ratio = abs_speed / RPM_LIMITS

    violation = ratio > 1.0

    count += 1

    # Topic is ~200 Hz.
    # Print about once per second.
    if count % 200 == 0:
        print()
        print("=" * 72)
        print("RM75 Joint Speed Watchdog")
        print("=" * 72)

        print(
            "speed [RPM] :",
            np.round(speeds, 3),
        )

        print(
            "limit [RPM] :",
            RPM_LIMITS,
        )

        print(
            "usage [%]   :",
            np.round(
                ratio * 100.0,
                1,
            ),
        )

        print(
            "max seen    :",
            np.round(
                max_seen,
                3,
            ),
        )

        print(
            "SAFE        :",
            not np.any(violation),
        )

    if np.any(violation):
        bad = np.where(violation)[0]

        rospy.logerr_throttle(
            0.2,
            "SPEED LIMIT EXCEEDED: joints=%s speeds=%s RPM",
            list(bad + 1),
            np.round(
                speeds[bad],
                3,
            ).tolist(),
        )


def main():
    rospy.init_node(
        "rm75_speed_watchdog",
        anonymous=True,
    )

    rospy.Subscriber(
        TOPIC,
        Joint_Speed,
        cb,
        queue_size=20,
    )

    print("=" * 72)
    print("RM75 SPEED WATCHDOG")
    print("=" * 72)
    print("MONITOR ONLY")
    print("NO ROBOT COMMANDS ARE PUBLISHED")
    print()
    print("Limits [RPM]:", RPM_LIMITS)
    print()

    rospy.spin()


if __name__ == "__main__":
    main()
