#!/usr/bin/env python3
"""Fresh-ACK contract tests; no ROS master or hardware is used."""

import threading
import unittest

from std_msgs.msg import Bool, Empty

from rm75_sampling_gui.ros.ros_interface import ROSInterface, ROSInterfaceError


class FakePublisher:
    def __init__(self):
        self.messages = []

    @staticmethod
    def get_num_connections():
        return 1

    def publish(self, message):
        self.messages.append(message)


class ROSInterfaceAckTest(unittest.TestCase):
    def setUp(self):
        self.interface = ROSInterface(
            {
                "ack_timeout_s": 0.2,
                "service_timeout_s": 0.2,
                "feedback_max_age_s": 1.0,
            }
        )
        self.interface._connected = True  # isolated unit-test fixture
        self.publisher = FakePublisher()

    def test_old_ack_is_not_accepted(self):
        self.interface._ack_callback("emergency", Bool(data=True))
        timer = threading.Timer(
            0.02,
            lambda: self.interface._ack_callback("emergency", Bool(data=True)),
        )
        timer.start()
        result = self.interface._publish_and_wait_ack(
            "emergency", self.publisher, Empty(), "test-stop"
        )
        timer.join()
        self.assertEqual(result["sequence"], 2)
        self.assertEqual(len(self.publisher.messages), 1)

    def test_fresh_negative_ack_is_rejected(self):
        timer = threading.Timer(
            0.02,
            lambda: self.interface._ack_callback("emergency", Bool(data=False)),
        )
        timer.start()
        with self.assertRaises(ROSInterfaceError):
            self.interface._publish_and_wait_ack(
                "emergency", self.publisher, Empty(), "test-stop"
            )
        timer.join()

    def test_missing_ack_times_out(self):
        with self.assertRaises(ROSInterfaceError):
            self.interface._publish_and_wait_ack(
                "emergency", self.publisher, Empty(), "test-stop"
            )


if __name__ == "__main__":
    unittest.main()
