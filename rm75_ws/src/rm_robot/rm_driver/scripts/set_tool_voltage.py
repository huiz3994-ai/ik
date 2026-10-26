#!/usr/bin/env python3
"""Set the RealMan tool connector voltage and verify the controller reply."""

import argparse
import threading
import time

import rospy
from std_msgs.msg import Bool, Byte


COMMAND_TOPIC = "/rm_driver/SetToolVoltage"
RESULT_TOPIC = "/rm_driver/Set_Tool_Voltage_Result"
VOLTAGE_TYPES = {0: 0, 12: 2, 24: 3}


def parse_args():
    parser = argparse.ArgumentParser(
        description="Set RM tool connector voltage (0V, 12V or 24V)."
    )
    parser.add_argument(
        "voltage",
        type=int,
        choices=sorted(VOLTAGE_TYPES),
        help="requested output voltage",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=5.0,
        help="seconds to wait for the driver and controller reply (default: 5)",
    )
    return parser.parse_args(rospy.myargv()[1:])


def main():
    args = parse_args()
    if args.timeout <= 0:
        raise SystemExit("--timeout must be greater than zero")

    rospy.init_node("set_tool_voltage", anonymous=True)
    reply_received = threading.Event()
    reply_ok = {"value": False}

    def result_callback(message):
        reply_ok["value"] = message.data
        reply_received.set()

    result_subscriber = rospy.Subscriber(
        RESULT_TOPIC, Bool, result_callback, queue_size=1
    )
    command_publisher = rospy.Publisher(COMMAND_TOPIC, Byte, queue_size=1)

    deadline = time.time() + args.timeout
    while (
        command_publisher.get_num_connections() == 0
        or result_subscriber.get_num_connections() == 0
    ) and not rospy.is_shutdown():
        if time.time() >= deadline:
            rospy.logerr(
                "rm_driver command/result topics are not both available: %s, %s",
                COMMAND_TOPIC,
                RESULT_TOPIC,
            )
            return 2
        rospy.sleep(0.05)

    protocol_value = VOLTAGE_TYPES[args.voltage]
    rospy.loginfo(
        "Setting tool connector to %dV (protocol value %d)",
        args.voltage,
        protocol_value,
    )
    command_publisher.publish(Byte(data=protocol_value))

    remaining = max(0.0, deadline - time.time())
    if not reply_received.wait(remaining):
        rospy.logerr("Timed out waiting for controller reply on %s", RESULT_TOPIC)
        result_subscriber.unregister()
        return 3

    result_subscriber.unregister()
    if not reply_ok["value"]:
        rospy.logerr("Controller rejected the tool-voltage request")
        return 4

    rospy.loginfo("Tool connector voltage set to %dV", args.voltage)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
