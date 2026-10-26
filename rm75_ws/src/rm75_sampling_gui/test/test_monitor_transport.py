#!/usr/bin/env python3
"""ROS transport test run by rostest on its own master; synthetic telemetry only."""
import json
import time
import unittest
import rospy
import rostest
import tf2_ros
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Float64, String
from rm_msgs.msg import Six_Force
from rm75_sampling_gui.ros.ros_interface import ROSInterface


class LiveMonitorTest(unittest.TestCase):
    def test_transport_discovery_tf_and_sensor_loss(self):
        rospy.set_param("/robot_description", '<robot name="test"><joint name="joint1"><limit lower="-2" upper="2"/></joint></robot>')
        interface = ROSInterface({"feedback_max_age_s": 0.4,
            "monitoring": {"graph_poll_s": 0.1, "graph_max_age_s": 2.0,
                           "nodes": {"driver": rospy.get_name()},
                           "image_topic": "/fixture/rgb"}})
        publishers = [
            rospy.Publisher("/joint_states", JointState, queue_size=1),
            rospy.Publisher("/rm_driver/UdpSixZeroForce", Six_Force, queue_size=1),
            rospy.Publisher("/rm75_touch_teleop/force_z", Float64, queue_size=1),
            rospy.Publisher("/rm75_touch_teleop/filtered_force_z", Float64, queue_size=1),
            rospy.Publisher("/rm75_touch_teleop/force_setpoint", Float64, queue_size=1),
            rospy.Publisher("/qr_task/vision_status", String, queue_size=1),
            rospy.Publisher("/qr_task/marker_observation", String, queue_size=1)]
        broadcaster = tf2_ros.TransformBroadcaster()
        try:
            self.assertTrue(interface.connect())
            # Discover a camera arriving after the HMI, not only at startup.
            camera = rospy.Publisher("/fixture/rgb", Image, queue_size=1)
            publishers.append(camera)
            deadline = time.monotonic() + 8.0
            while time.monotonic() < deadline:
                stamp = rospy.Time.now()
                joints = JointState(name=["joint{}".format(i) for i in range(1, 8)],
                                    position=[0.1]*7)
                joints.header.stamp = stamp
                publishers[0].publish(joints)
                publishers[1].publish(Six_Force(force_Fx=1, force_Fy=2, force_Fz=3,
                                               force_Mx=4, force_My=5, force_Mz=6))
                for pub, value in zip(publishers[2:5], (-3.0, -2.5, -5.0)):
                    pub.publish(Float64(data=value))
                publishers[5].publish(String(data=json.dumps(
                    {"detections": [{"slot_id": 4, "stable": True}]})))
                publishers[6].publish(String(data=json.dumps({
                    "slot_id": 4, "stamp": stamp.to_sec(), "frame_id": "base_link",
                    "translation": [0.1, 0.2, 0.3], "quaternion_xyzw": [0, 0, 0, 1]})))
                image = Image(width=2, height=1, step=6, encoding="rgb8",
                              data=bytes([255, 0, 0, 0, 255, 0]))
                image.header.stamp = stamp
                camera.publish(image)
                transform = TransformStamped()
                transform.header.stamp = stamp
                transform.header.frame_id = "base_link"
                transform.child_frame_id = "Link7"
                transform.transform.translation.z = 0.5
                transform.transform.rotation.w = 1
                broadcaster.sendTransform(transform)
                data = interface.telemetry_snapshot()
                if all(data.get("fresh", {}).get(key) for key in
                       ("joints", "tcp", "image", "wrench", "force_z", "marker_4")):
                    break
                time.sleep(0.03)
            self.assertTrue(data["health"]["master"])
            self.assertTrue(data["health"]["driver"])
            self.assertEqual(data["image_topic"], "/fixture/rgb")
            self.assertEqual(data["wrench"]["Tz"], 6)
            self.assertEqual(data["tcp"]["z"], 0.5)
            self.assertTrue(data["fresh"]["image"])
            self.assertEqual(data["joint_limits"]["joint1"], (-2, 2))
            for publisher in publishers:
                publisher.unregister()
            publishers.clear()
            time.sleep(0.7)
            data = interface.telemetry_snapshot()
            self.assertFalse(data["fresh"]["wrench"])
            self.assertFalse(data["fresh"]["force_z"])
            self.assertFalse(data["fresh"]["joints"])
            self.assertFalse(data["fresh"]["image"])
        finally:
            for publisher in publishers:
                publisher.unregister()
            interface.shutdown()


if __name__ == "__main__":
    rospy.init_node("field_station_fixture")
    rostest.rosrun("rm75_sampling_gui", "monitor_transport", LiveMonitorTest)

