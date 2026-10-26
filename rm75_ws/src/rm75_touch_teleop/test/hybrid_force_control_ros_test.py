#!/usr/bin/env python3
"""Isolated mock plant. No subscriptions/publications to any real driver command."""
import math
import threading
import time
import unittest
import rospy
import rostest
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Vector3Stamped
from sensor_msgs.msg import JointState, Joy
from std_msgs.msg import Bool, String, Empty, UInt16, Float64MultiArray
from rm_msgs.msg import Six_Force, JointPos
from trajectory_msgs.msg import JointTrajectory


class Plant:
    def __init__(self, name):
        self.name = name
        self.grey = False
        self.white = False
        self.angle = 0.0
        self.force = 0.0
        self.force_x = 0.0
        self.force_y = 0.0
        self.offset = [0., 0., 0.]
        self.q = [0., .8395, 0., 1.4442, 0., .8513, 0.]
        self.mode = ''
        self.active = False
        self.commands = []
        self.targets = []
        self.stops = 0
        self.haptic_commands = []
        self.enabled = set(['pose', 'joy', 'joints', 'force', 'increment', 'coordinate'])
        self.coordinate = 0
        self.follow = True
        self.publishers = {}
        for key, msg in [('pose', PoseStamped), ('joy', Joy), ('joints', JointState),
                         ('force', Six_Force), ('increment', Float64MultiArray), ('coordinate', UInt16)]:
            self.publishers[key] = rospy.Publisher('/test/'+name+'/'+key, msg, queue_size=1)
        rospy.Subscriber('/test_'+name+'/control_mode', String, lambda m: setattr(self, 'mode', m.data))
        rospy.Subscriber('/test_'+name+'/active', Bool, lambda m: setattr(self, 'active', m.data))
        rospy.Subscriber('/test_'+name+'/target_pose', PoseStamped, lambda m: self.targets.append(m))
        rospy.Subscriber('/test/'+name+'/stop', Empty, self.stop)
        rospy.Subscriber('/test/'+name+'/command',
                         JointPos if name in ('joint', 'observe') else JointTrajectory,
                         self.command)
        rospy.Subscriber('/test_'+name+'/haptic_device_command', Vector3Stamped,
                         lambda m: self.haptic_commands.append(m))

    def stop(self, _):
        self.stops += 1

    def command(self, msg):
        q = list(msg.joint if self.name in ('joint', 'observe') else msg.points[0].positions)
        self.commands.append(q)
        if self.follow:
            self.q = q

    def tick(self):
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.pose.orientation.w = math.cos(self.angle/2)
        pose.pose.orientation.x = math.sin(self.angle/2)
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = self.offset
        joy = Joy(buttons=[int(self.grey), int(self.white)])
        joints = JointState(name=['joint'+str(i) for i in range(1, 8)], position=self.q)
        msgs = dict(pose=pose, joy=joy, joints=joints,
                    force=Six_Force(force_Fx=self.force_x,
                                    force_Fy=self.force_y,
                                    force_Fz=self.force),
                    increment=Float64MultiArray(data=self.offset), coordinate=UInt16(data=self.coordinate))
        for key in list(self.enabled):
            self.publishers[key].publish(msgs[key])


class Integration(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.plants = [Plant(n) for n in
                      ('joint', 'trajectory', 'plain', 'observe', 'z_only')]
        cls.running = True
        def loop():
            while cls.running and not rospy.is_shutdown():
                for p in cls.plants:
                    p.tick()
                time.sleep(.01)
        cls.thread = threading.Thread(target=loop, daemon=True)
        cls.thread.start()
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline:
            ready = all(p.publishers[k].get_num_connections() > 0
                        for p in cls.plants for k in ('pose', 'joy', 'joints', 'increment'))
            ready = ready and all(p.publishers['coordinate'].get_num_connections() > 0
                                  for p in (cls.plants[0], cls.plants[3]))
            ready = ready and all(p.publishers['force'].get_num_connections() > 0
                                  for p in (cls.plants[0], cls.plants[1],
                                            cls.plants[3], cls.plants[4]))
            if ready:
                break
            time.sleep(.02)
        time.sleep(.3)

    @classmethod
    def tearDownClass(cls):
        cls.running = False
        cls.thread.join(2)

    def wait(self, pred, timeout=3):
        until = time.monotonic()+timeout
        while time.monotonic() < until:
            if pred():
                return
            time.sleep(.01)
        self.assertTrue(pred(), 'Timed out waiting for control condition')

    def engage(self, p):
        p.grey = False
        time.sleep(.12)
        p.grey = True
        self.wait(lambda: p.active)
        count = len(p.commands)
        self.wait(lambda: len(p.commands) > count+2)

    def stopped(self, p):
        self.wait(lambda: p.mode == 'SAFE_STOP' and not p.active)
        time.sleep(.08)
        count = len(p.commands)
        time.sleep(.12)
        self.assertEqual(len(p.commands), count)

    def test_control_and_failures(self):
        observer = self.plants[3]
        observer.force = -101.
        self.wait(lambda: observer.mode == 'SAFE_STOP')
        time.sleep(.15)
        self.assertFalse(observer.active)
        self.assertEqual(observer.stops, 0)
        self.assertEqual(len(observer.commands), 0)

        for p in self.plants[:2]:
            self.engage(p)
            self.assertEqual(p.mode, 'POSITION_MODE')
            p.force = -15.
            self.wait(lambda: p.mode == 'CONTACT_MODE')
            time.sleep(.12)
            z = p.targets[-1].pose.position.z
            # Touch Y maps to robot Z and is ignored in contact mode.
            p.offset[1] += 100.
            time.sleep(.25)
            self.assertTrue(p.active)
            self.assertLess(abs(p.targets[-1].pose.position.z-z), .004)
            # X/Y remains controlled; Touch +Z maps into robot -X.
            x = p.targets[-1].pose.position.x
            p.offset[2] += 2.
            time.sleep(.12)
            self.assertLess(p.targets[-1].pose.position.x, x-.001)
            # White-button orientation remains available during contact.
            p.white = True
            time.sleep(.08)
            before = p.targets[-1].pose.orientation
            p.angle += .01
            time.sleep(.12)
            after = p.targets[-1].pose.orientation
            self.assertGreater(sum(abs(getattr(after,k)-getattr(before,k)) for k in ('x','y','z','w')), .001)
            p.white = False
            p.force = 0.
            self.wait(lambda: p.mode == 'POSITION_MODE')
            z = p.targets[-1].pose.position.z
            time.sleep(.15)
            self.assertLess(abs(p.targets[-1].pose.position.z-z), .003)
            p.offset[1] += 2.
            time.sleep(.12)
            self.assertGreater(p.targets[-1].pose.position.z, z+.001)
            # Every actually emitted command respects the original joint velocity bound.
            for a, b in zip(p.commands, p.commands[1:]):
                self.assertLessEqual(max(abs(x-y) for x,y in zip(a,b)), .00401)
            p.force = -101.
            self.stopped(p)
            if p.name == 'joint':
                self.assertGreater(p.stops, 0)
            p.force = 0.
            time.sleep(.2)
            self.assertFalse(p.active)  # no automatic recovery while held
            for missing in ('force', 'pose', 'joy', 'increment', 'joints') + (('coordinate',) if p.name == 'joint' else ()):
                self.engage(p)
                p.enabled.remove(missing)
                self.stopped(p)
                p.enabled.add(missing)
                time.sleep(.12)
            self.engage(p)
            p.force = float('nan')
            self.stopped(p)
            p.force = 0.
            time.sleep(.12)
            if p.name == 'joint':
                self.engage(p)
                p.coordinate = 1
                self.stopped(p)
                p.coordinate = 0
                time.sleep(.12)
            self.engage(p)
            # Far outside reachable workspace, but inside test displacement guard.
            p.offset[0] += 3000.
            time.sleep(.04)
            count = len(p.commands)
            self.stopped(p)
            self.assertEqual(len(p.commands), count)
            p.offset[0] -= 3000.
            p.grey = False
        # Ordinary mode remains usable without any force publisher.
        p = self.plants[2]
        p.enabled.remove('force')
        self.engage(p)
        z = p.targets[-1].pose.position.z
        p.offset[1] += 2.
        time.sleep(.2)
        self.assertEqual(p.mode, 'POSITION_MODE')
        self.assertGreater(p.targets[-1].pose.position.z, z+.001)
        p.grey = False
        self.wait(lambda: not p.active)

        # White is an independent orientation clutch: it must engage without
        # grey, rotate the target, and leave translation frozen.
        p.white = True
        self.wait(lambda: p.active)
        time.sleep(.08)
        before = p.targets[-1].pose
        before_xyz = (before.position.x, before.position.y, before.position.z)
        before_q = before.orientation
        p.angle += .01
        time.sleep(.15)
        after = p.targets[-1].pose
        self.assertGreater(sum(abs(getattr(after.orientation, k) -
                                   getattr(before_q, k))
                               for k in ('x', 'y', 'z', 'w')), .001)
        self.assertLess(sum(abs(a - b) for a, b in
                            zip((after.position.x, after.position.y,
                                 after.position.z), before_xyz)), .001)
        p.white = False
        self.wait(lambda: not p.active)
        # Feature-off regression: the new haptic path emits no device command,
        # while all legacy target/trajectory assertions above still pass.
        self.assertEqual(len(p.haptic_commands), 0)

    def test_z_only_hybrid_and_haptic_contract(self):
        p = self.plants[4]
        self.engage(p)
        self.assertEqual(p.mode, 'POSITION_MODE')
        before = p.targets[-1].pose
        fixed_xy = (before.position.x, before.position.y)
        session_z = before.position.z
        fixed_q = tuple(getattr(before.orientation, key)
                        for key in ('x', 'y', 'z', 'w'))

        # Touch X/Z would normally map to robot Y/X, and white would normally
        # rotate.  The dedicated profile must suppress all three operations.
        p.white = True
        p.angle += .08
        p.offset[0] += 3.
        p.offset[2] += 3.
        time.sleep(.2)
        locked = p.targets[-1].pose
        self.assertLess(abs(locked.position.x-fixed_xy[0]), 1e-5)
        self.assertLess(abs(locked.position.y-fixed_xy[1]), 1e-5)
        self.assertLess(sum(abs(getattr(locked.orientation, key)-value)
                            for key, value in zip(('x', 'y', 'z', 'w'), fixed_q)),
                        1e-5)

        # Touch Y maps to robot Z and remains the sole position DOF before
        # contact.
        start_z = locked.position.z
        p.offset[1] += 2.
        self.wait(lambda: p.targets[-1].pose.position.z > start_z+.0006)
        moved = p.targets[-1].pose
        self.assertLess(abs(moved.position.x-fixed_xy[0]), 1e-5)
        self.assertLess(abs(moved.position.y-fixed_xy[1]), 1e-5)

        # Virtual contact transfers Z ownership to the force loop.  The raw
        # robot-Z load is reflected only on Touch Y and remains under 0.5 N.
        p.force_x = 4.
        p.force_y = 3.
        p.force = -5.
        self.wait(lambda: p.mode == 'CONTACT_MODE')
        self.wait(lambda: p.haptic_commands and
                  abs(p.haptic_commands[-1].vector.y) > .02)
        feedback = p.haptic_commands[-1].vector
        self.assertLess(abs(feedback.x), 1e-8)
        self.assertLess(abs(feedback.z), 1e-8)
        self.assertLessEqual(math.sqrt(feedback.x**2 + feedback.y**2 +
                                       feedback.z**2), .5001)

        contact_xy = (p.targets[-1].pose.position.x,
                      p.targets[-1].pose.position.y)
        p.offset[1] += 100.
        time.sleep(.2)
        contact = p.targets[-1].pose
        self.assertLess(abs(contact.position.x-contact_xy[0]), 1e-5)
        self.assertLess(abs(contact.position.y-contact_xy[1]), 1e-5)
        self.assertLess(sum(abs(getattr(contact.orientation, key)-value)
                            for key, value in zip(('x', 'y', 'z', 'w'), fixed_q)),
                        1e-5)

        p.grey = False
        p.white = False
        p.force_x = 0.
        p.force_y = 0.
        self.wait(lambda: not p.active)
        self.wait(lambda: p.haptic_commands and
                  abs(p.haptic_commands[-1].vector.y) < 1e-6)

        # A large free-space hand displacement is rate limited to 2 mm/s and
        # cannot ratchet past the node-lifetime travel envelope by repeatedly
        # releasing and re-engaging the grey clutch.
        p.force = 0.
        time.sleep(.15)
        self.engage(p)
        rate_start = p.targets[-1].pose.position.z
        p.offset[1] += 100.
        time.sleep(.3)
        rate_limited = p.targets[-1].pose.position.z
        self.assertLessEqual(rate_limited-rate_start, .0008)
        self.wait(lambda: p.targets[-1].pose.position.z > session_z+.0035,
                  timeout=5.)
        # The contact phase may have left Z near the negative side of the
        # envelope; allow enough time at 2 mm/s to reach the positive bound.
        time.sleep(3.)
        boundary_z = p.targets[-1].pose.position.z
        all_session_z = [message.pose.position.z for message in p.targets]
        self.assertLessEqual(max(all_session_z)-min(all_session_z), .00801)
        p.grey = False
        self.wait(lambda: not p.active)

        self.engage(p)
        p.offset[1] += 100.
        time.sleep(.5)
        bounded = p.targets[-1].pose
        self.assertLessEqual(bounded.position.z, boundary_z+1e-5)
        self.assertLess(abs(bounded.position.x-fixed_xy[0]), 1e-5)
        self.assertLess(abs(bounded.position.y-fixed_xy[1]), 1e-5)
        self.assertLess(sum(abs(getattr(bounded.orientation, key)-value)
                            for key, value in zip(('x', 'y', 'z', 'w'), fixed_q)),
                        1e-5)
        self.assertEqual(p.mode, 'POSITION_MODE')
        p.grey = False
        self.wait(lambda: not p.active)


if __name__ == '__main__':
    rospy.init_node('hybrid_ros_test')
    rostest.rosrun('rm75_touch_teleop', 'hybrid_ros', Integration)
