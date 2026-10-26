"""Read-only telemetry, bounded ROS graph polling, TF and image discovery."""
import copy
import logging
import math
import os
import threading
import time
import xml.etree.ElementTree as ET
import xmlrpc.client
from typing import Any, Dict
import rospy
import tf2_ros
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import Image
from std_msgs.msg import String
from rm_msgs.msg import Six_Force

LOG = logging.getLogger(__name__)
JOINTS = ["joint{}".format(i) for i in range(1, 8)]


class TimeoutTransport(xmlrpc.client.Transport):
    def make_connection(self, host: str) -> Any:
        connection = super().make_connection(host)
        connection.timeout = 0.4
        return connection


class Monitor:
    """Receives telemetry without publishing any control message."""

    def __init__(self, config: Dict[str, Any]) -> None:
        self.config = config
        self.options = config.get("monitoring", {})
        self.max_age = float(config.get("feedback_max_age_s", 1.0))
        self.lock = threading.RLock()
        self.stop_event = threading.Event()
        self.thread = None
        self.subscribers = []
        self.image_subscriber = None
        self.tf_buffer = None
        self.tf_listener = None
        self.values = {}
        self.received = {}
        self.sequences = {}
        self.source_stamps = {}
        self.health = {}
        self.limits = {}
        self.image = None
        self.image_topic = ""
        self.master_seen = 0.0
        self.graph_checked = 0.0

    def put(self, name: str, value: Any, stamp: float = None) -> None:
        with self.lock:
            self.values[name] = value
            self.received[name] = time.monotonic()
            self.sequences[name] = self.sequences.get(name, 0) + 1
            self.source_stamps[name] = stamp

    def joints(self, message: Any) -> None:
        if len(message.name) != len(message.position):
            return
        values = dict(zip(message.name, message.position))
        if not all(name in values and math.isfinite(values[name]) for name in JOINTS):
            return
        stamp = message.header.stamp.to_sec()
        age = rospy.Time.now().to_sec() - stamp
        if stamp <= 0 or not 0 <= age <= self.max_age:
            return
        self.put("joints", {name: float(values[name]) for name in JOINTS}, stamp)
        # Only label velocities as measured when the driver supplied a complete
        # finite name-mapped array. Missing data remains explicitly unavailable;
        # no numerical estimate is presented as hardware feedback.
        if len(message.velocity) == len(message.name):
            velocity = dict(zip(message.name, message.velocity))
            if all(name in velocity and math.isfinite(velocity[name]) for name in JOINTS):
                self.put(
                    "joint_velocities",
                    {name: float(velocity[name]) for name in JOINTS},
                    stamp,
                )
                self.put("joint_velocity_source", "measured", stamp)
                return
        self.put("joint_velocities", {}, stamp)
        self.put("joint_velocity_source", "unavailable", stamp)

    def wrench(self, message: Any) -> None:
        values = [getattr(message, field) for field in
                  ("force_Fx", "force_Fy", "force_Fz", "force_Mx", "force_My", "force_Mz")]
        if all(math.isfinite(value) for value in values):
            # Six_Force has no header. This timestamp is receipt time, not sensor time.
            self.put("wrench", dict(zip(("Fx", "Fy", "Fz", "Tx", "Ty", "Tz"), values)))

    def observation(self, message: Any) -> None:
        import json
        try:
            value = json.loads(message.data)
            slot = int(value["slot_id"])
            stamp = float(value["stamp"])
            numbers = value["translation"] + value["quaternion_xyzw"]
            if len(numbers) != 7 or not all(math.isfinite(float(x)) for x in numbers):
                return
            if not 0 <= rospy.Time.now().to_sec() - stamp <= self.max_age:
                return
            self.put("marker_{}".format(slot), value, stamp)
        except (ValueError, TypeError, KeyError):
            LOG.warning("忽略无效Mark位姿")

    def camera(self, message: Any) -> None:
        stamp = message.header.stamp.to_sec()
        if stamp <= 0 or not 0 <= rospy.Time.now().to_sec() - stamp <= self.max_age:
            return
        if message.width * message.height > int(self.options.get("image_max_pixels", 4096000)):
            return
        with self.lock:
            self.image = message
        self.put("image", True, stamp)

    def parse_limits(self, xml: str) -> None:
        result = {}
        for joint in ET.fromstring(xml).findall("joint"):
            name = joint.get("name")
            limit = joint.find("limit")
            if name in JOINTS and limit is not None:
                lower, upper = float(limit.get("lower")), float(limit.get("upper"))
                if math.isfinite(lower) and math.isfinite(upper) and lower < upper:
                    result[name] = (lower, upper)
        with self.lock:
            self.limits = result

    def start(self) -> None:
        if self.thread is not None:
            return
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.subscribers = [
            rospy.Subscriber(self.options.get("wrench_topic", "/rm_driver/UdpSixZeroForce"),
                             Six_Force, self.wrench, queue_size=1),
            rospy.Subscriber("/qr_task/marker_observation", String,
                             self.observation, queue_size=5),
            rospy.Subscriber("/qr_task/five_targets/status", String,
                             lambda message: self._json(
                                 "five_target_status", message), queue_size=5),
            rospy.Subscriber("/qr_task/five_targets/targets", String,
                             lambda message: self._json(
                                 "five_targets", message), queue_size=5),
        ]
        self.thread = threading.Thread(target=self._run, name="hmi-monitor", daemon=True)
        self.thread.start()

    def _json(self, key: str, message: Any) -> None:
        import json
        try:
            value = json.loads(message.data)
            if isinstance(value, dict):
                self.put(key, value)
        except (TypeError, ValueError):
            LOG.warning("忽略无效%s JSON", key)

    def _proxy(self, uri: str) -> Any:
        return xmlrpc.client.ServerProxy(uri, transport=TimeoutTransport())

    def _graph(self) -> None:
        caller = rospy.get_name()
        health = {}
        try:
            with self._proxy(os.environ.get("ROS_MASTER_URI", "http://localhost:11311")) as master:
                code, _, _ = master.getPid(caller)
                if code != 1:
                    raise RuntimeError("master getPid failed")
                self.master_seen = time.monotonic()
                health["master"] = True
                nodes = self.options.get("nodes", {
                    "driver": "/rm_driver", "touch": "/Geomagic/Geomagic",
                    "qr": "/rm75_qr_task_server", "marker": "/rm75_qr_marker"})
                for key, node in nodes.items():
                    if self.stop_event.is_set():
                        return
                    try:
                        code, _, uri = master.lookupNode(caller, node)
                        if code != 1:
                            raise RuntimeError("node missing")
                        with self._proxy(uri) as remote:
                            health[key] = remote.getPid(caller)[0] == 1
                    except Exception:
                        health[key] = False
                code, _, xml = master.getParam(caller, "/robot_description")
                if code == 1:
                    self.parse_limits(xml)
                code, _, topics = master.getPublishedTopics(caller, "")
                if code == 1:
                    candidates = sorted(name for name, kind in topics
                                        if kind == "sensor_msgs/Image"
                                        and ("color" in name or "rgb" in name))
                    preferred = self.options.get("image_topic", "/camera/color/image_raw")
                    names = {name for name, kind in topics if kind == "sensor_msgs/Image"}
                    topic = preferred if preferred in names else (candidates[0] if candidates else "")
                    if topic != self.image_topic:
                        if self.image_subscriber:
                            self.image_subscriber.unregister()
                        self.image_subscriber = None
                        self.image_topic = topic
                        if topic:
                            self.image_subscriber = rospy.Subscriber(
                                topic, Image, self.camera, queue_size=1,
                                buff_size=16 * 1024 * 1024)
        except Exception:
            health = {"master": False}
        with self.lock:
            self.health = health
            self.graph_checked = time.monotonic()

    def _tf(self) -> None:
        try:
            base = self.options.get("base_frame", "base_link")
            tip = self.options.get("tip_frame", "Link7")
            transform = self.tf_buffer.lookup_transform(base, tip, rospy.Time(0))
            stamp = transform.header.stamp.to_sec()
            if not 0 <= rospy.Time.now().to_sec() - stamp <= self.max_age:
                return
            t, q = transform.transform.translation, transform.transform.rotation
            rpy = euler_from_quaternion([q.x, q.y, q.z, q.w])
            values = [t.x, t.y, t.z] + list(rpy)
            if all(math.isfinite(x) for x in values):
                self.put("tcp", dict(zip(("x", "y", "z", "rx", "ry", "rz"), values)), stamp)
        except (tf2_ros.TransformException, ValueError):
            pass

    def _run(self) -> None:
        next_graph = 0.0
        while not self.stop_event.is_set() and not rospy.is_shutdown():
            if time.monotonic() >= next_graph:
                self._graph()
                next_graph = time.monotonic() + float(self.options.get("graph_poll_s", 2.0))
            self._tf()
            self.stop_event.wait(0.1)

    def snapshot(self) -> Dict[str, Any]:
        now = time.monotonic()
        with self.lock:
            result = copy.deepcopy(self.values)
            result["ages"] = {name: now - value for name, value in self.received.items()}
            result["fresh"] = {name: age <= self.max_age for name, age in result["ages"].items()}
            # Receipt of a fresh JointState does not make an absent velocity
            # array valid hardware feedback. The RM75 driver currently emits
            # position-only JointState messages at 200 Hz.
            if result.get("joint_velocity_source") != "measured":
                result["fresh"]["joint_velocities"] = False
            graph_fresh = now - self.graph_checked < float(self.options.get("graph_max_age_s", 6.0))
            result["health"] = {key: bool(value and graph_fresh) for key, value in self.health.items()}
            result["joint_limits"] = dict(self.limits)
            result["sequences"] = dict(self.sequences)
            result["source_stamps"] = dict(self.source_stamps)
            result["image_message"] = self.image  # immutable ROS message, no deep copy of pixels
            result["image_topic"] = self.image_topic
            result["timestamp"] = time.time()
            return result

    def close(self) -> None:
        self.stop_event.set()
        if self.thread:
            self.thread.join(timeout=3.0)
        for subscriber in self.subscribers:
            subscriber.unregister()
        if self.image_subscriber:
            self.image_subscriber.unregister()
        if self.tf_listener:
            self.tf_listener.unregister()
