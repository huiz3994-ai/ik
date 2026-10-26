"""Mockable interfaces and ROS adapters used by the task state machine."""

import copy
import threading
import time
import numpy as np

from .pose_manager import serialize_transform, transform_matrix, transform_parts


class AdapterError(RuntimeError):
    pass


class MockMotionAdapter(object):
    def __init__(self, joints=None, tcp=None, fail_action=None):
        self.joints = list(joints if joints is not None else [0.0] * 7)
        self.tcp = np.array(tcp if tcp is not None else np.eye(4), dtype=float)
        self.fail_action = fail_action
        self.actions = []

    def _record(self, name, value=None):
        self.actions.append((name, copy.deepcopy(value)))
        if self.fail_action == name:
            raise AdapterError('injected {} failure'.format(name))

    def snapshot(self):
        self._record('snapshot')
        self.last_snapshot = {'joints': list(self.joints), 'tcp': self.tcp.copy(),
                              'timestamp': time.time()}
        return copy.deepcopy(self.last_snapshot)

    def preview_sequence(self, poses, final_joints):
        self._record('preview_sequence', [np.asarray(p).copy() for p in poses])
        if len(final_joints) != 7:
            raise AdapterError('expected seven return joints')
        return {'segments': len(poses) + 1, 'collision_checked': False,
                'executed': False, 'mock': True,
                'note': 'Mock adapter does not perform geometric collision checking'}

    def move_pose(self, pose, label):
        self._record('move_' + label, np.asarray(pose).copy())
        self.tcp = np.asarray(pose).copy()

    def move_joints(self, joints, label):
        self._record('move_' + label, joints)
        self.joints = list(joints)
        if hasattr(self, 'last_snapshot') and joints == self.last_snapshot['joints']:
            self.tcp = self.last_snapshot['tcp'].copy()

    def verify(self, snapshot, joint_tolerance, translation_tolerance,
               rotation_tolerance):
        self._record('verify')
        if max(abs(a-b) for a, b in zip(self.joints, snapshot['joints'])) > joint_tolerance:
            raise AdapterError('joint return tolerance exceeded')
        if np.linalg.norm(self.tcp[:3, 3] - snapshot['tcp'][:3, 3]) > translation_tolerance:
            raise AdapterError('TCP translation return tolerance exceeded')
        from .pose_manager import rotation_distance
        if rotation_distance(self.tcp, snapshot['tcp']) > rotation_tolerance:
            raise AdapterError('TCP rotation return tolerance exceeded')

    def verify_pose(self, target, translation_tolerance, rotation_tolerance):
        from .pose_manager import rotation_distance
        if np.linalg.norm(self.tcp[:3, 3] - target[:3, 3]) > translation_tolerance:
            raise AdapterError('target TCP translation tolerance exceeded')
        if rotation_distance(self.tcp, target) > rotation_tolerance:
            raise AdapterError('target TCP rotation tolerance exceeded')

    def attach(self, slot_id, object_pose, object_config):
        self._record('attach', {'slot_id': slot_id,
                                'pose': np.asarray(object_pose).copy(),
                                'object': object_config})

    def add_world_object(self, slot_id, object_pose, object_config):
        self._record('add_world_object', {'slot_id': slot_id,
                                          'pose': np.asarray(object_pose).copy(),
                                          'object': object_config})

    def set_target_contact(self, slot_id, allow, object_config):
        self._record('target_contact', {'slot_id': slot_id, 'allow': bool(allow)})

    def detach(self, slot_id, object_pose, object_config):
        self._record('detach', {'slot_id': slot_id,
                                'pose': np.asarray(object_pose).copy(),
                                'object': object_config})

    def stop(self):
        self._record('stop')

class MockVoltageAdapter(object):
    def __init__(self, fail=False):
        self.fail = fail
        self.commands = []

    def set_voltage(self, voltage):
        self.commands.append(int(voltage))
        if self.fail:
            raise AdapterError('injected voltage ACK failure')
        return True

    def power_status(self):
        return {'last_command_voltage': self.commands[-1] if self.commands else None,
                'last_ack': False if self.fail else (True if self.commands else None)}


class MockHoldFeedback(object):
    def __init__(self, holding=True, released=True):
        self.holding_value = holding
        self.released_value = released

    def holding(self):
        return bool(self.holding_value)

    def released(self):
        return bool(self.released_value)

    def feedback_status(self):
        return {'fresh': True, 'holding': bool(self.holding_value),
                'released': bool(self.released_value), 'source': 'mock'}


class RosVoltageAdapter(object):
    """Uses the existing rm_driver command/ACK interface, never a new driver."""
    PROTOCOL = {0: 0, 12: 2, 24: 3}

    def __init__(self, rospy, timeout=5.0):
        from std_msgs.msg import Bool, Byte
        self.rospy = rospy
        self.Byte = Byte
        self.timeout = float(timeout)
        self.condition = threading.Condition()
        self.sequence = 0
        self.last_result = False
        self.last_wall_time = 0.0
        self.publisher = rospy.Publisher('/rm_driver/SetToolVoltage', Byte,
                                         queue_size=1)
        self.subscriber = rospy.Subscriber(
            '/rm_driver/Set_Tool_Voltage_Result', Bool, self._reply,
            queue_size=1)

    def _reply(self, message):
        with self.condition:
            self.sequence += 1
            self.last_result = bool(message.data)
            self.last_wall_time = time.monotonic()
            self.condition.notify_all()

    def set_voltage(self, voltage):
        voltage = int(voltage)
        if voltage not in self.PROTOCOL:
            raise AdapterError('unsupported tool voltage {}'.format(voltage))
        deadline = time.monotonic() + self.timeout
        while (self.publisher.get_num_connections() == 0 or
               self.subscriber.get_num_connections() == 0):
            if self.rospy.is_shutdown() or time.monotonic() >= deadline:
                raise AdapterError('tool-voltage command/result topic unavailable')
            time.sleep(0.05)
        with self.condition:
            baseline = self.sequence
            sent_at = time.monotonic()
            self.publisher.publish(self.Byte(data=self.PROTOCOL[voltage]))
            while self.sequence <= baseline or self.last_wall_time < sent_at:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise AdapterError('fresh tool-voltage ACK timeout')
                self.condition.wait(remaining)
            if not self.last_result:
                raise AdapterError('controller rejected tool-voltage command')
        return True

    def power_status(self):
        with self.condition:
            return {'ack_sequence': self.sequence,
                    'last_ack': self.last_result if self.sequence else None,
                    'last_ack_monotonic': self.last_wall_time or None,
                    'note': 'ACK confirms command acceptance, not measured output voltage'}


class RosHoldFeedback(object):
    """Independent object-present feedback; voltage ACK is deliberately ignored."""
    def __init__(self, rospy, topic, timeout=0.5):
        from std_msgs.msg import Bool
        self.rospy = rospy
        self.timeout = float(timeout)
        self.value = None
        self.stamp = None
        self.lock = threading.Lock()
        self.subscriber = rospy.Subscriber(topic, Bool, self._callback,
                                           queue_size=1)

    def _callback(self, message):
        with self.lock:
            self.value = bool(message.data)
            self.stamp = time.monotonic()

    def _fresh(self):
        with self.lock:
            if self.stamp is None or time.monotonic() - self.stamp > self.timeout:
                raise AdapterError('object feedback missing or stale')
            return self.value

    def holding(self):
        return self._fresh()

    def released(self):
        return not self._fresh()

    def feedback_status(self):
        with self.lock:
            age = None if self.stamp is None else time.monotonic() - self.stamp
            return {'fresh': age is not None and age <= self.timeout,
                    'age_s': age, 'holding': self.value,
                    'source': self.subscriber.resolved_name}


class MoveItMotionAdapter(object):
    """Thin adapter over the existing MoveIt stack and its trajectory bridge."""
    def __init__(self, group_name='arm', tcp_link='grasp_tcp', base_frame='base_link'):
        import moveit_commander
        import rospy
        from geometry_msgs.msg import PoseStamped
        moveit_commander.roscpp_initialize([])
        self.rospy = rospy
        self.PoseStamped = PoseStamped
        self.group = moveit_commander.MoveGroupCommander(group_name)
        self.scene = moveit_commander.PlanningSceneInterface(synchronous=True)
        self.tcp_link = tcp_link
        self.base_frame = base_frame
        self.group.set_end_effector_link(tcp_link)
        self.group.set_pose_reference_frame(base_frame)
        expected = ['joint{}'.format(index) for index in range(1, 8)]
        if list(self.group.get_active_joints()) != expected:
            raise AdapterError('MoveIt arm group is not ordered joint1..joint7')

    def _pose(self, transform):
        t, q = transform_parts(transform)
        pose = self.PoseStamped()
        pose.header.frame_id = self.base_frame
        pose.header.stamp = self.rospy.Time.now()
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = t
        (pose.pose.orientation.x, pose.pose.orientation.y,
         pose.pose.orientation.z, pose.pose.orientation.w) = q
        return pose

    @staticmethod
    def _plan_value(result):
        # Noetic Python API returns (success, trajectory, planning_time, error).
        if isinstance(result, tuple):
            if not result[0]:
                raise AdapterError('MoveIt planning failed: {}'.format(result[-1]))
            return result[1]
        if not result or not result.joint_trajectory.points:
            raise AdapterError('MoveIt planning returned an empty trajectory')
        return result

    def snapshot(self):
        joints = self.group.get_current_joint_values()
        if len(joints) != 7 or not np.all(np.isfinite(joints)):
            raise AdapterError('joint feedback is not a finite seven-joint state')
        pose = self.group.get_current_pose(self.tcp_link).pose
        tcp = transform_matrix(
            [pose.position.x, pose.position.y, pose.position.z],
            [pose.orientation.x, pose.orientation.y,
             pose.orientation.z, pose.orientation.w])
        return {'joints': list(joints), 'tcp': tcp,
                'timestamp': self.rospy.Time.now().to_sec()}

    def preview_sequence(self, poses, final_joints):
        from moveit_msgs.msg import RobotState
        from sensor_msgs.msg import JointState
        trajectories = []
        self.group.set_start_state_to_current_state()
        for pose in poses:
            self.group.set_pose_target(self._pose(pose), self.tcp_link)
            trajectory = self._plan_value(self.group.plan())
            trajectories.append(trajectory)
            self.group.clear_pose_targets()
            end = trajectory.joint_trajectory.points[-1]
            start = RobotState()
            start.is_diff = True
            start.joint_state = JointState(
                name=list(trajectory.joint_trajectory.joint_names),
                position=list(end.positions))
            self.group.set_start_state(start)
        self.group.set_joint_value_target(final_joints)
        trajectories.append(self._plan_value(self.group.plan()))
        self.group.set_start_state_to_current_state()
        return {'segments': len(trajectories), 'collision_checked': True,
                'executed': False,
                'note': 'Preview chains each planned endpoint; execution '
                        'replans every segment from measured state.'}

    def move_pose(self, pose, label):
        self.group.set_start_state_to_current_state()
        self.group.set_pose_target(self._pose(pose), self.tcp_link)
        plan = self._plan_value(self.group.plan())
        self.group.clear_pose_targets()
        if not self.group.execute(plan, wait=True):
            raise AdapterError('{} execution failed'.format(label))
        self.group.stop()

    def move_joints(self, joints, label):
        self.group.set_start_state_to_current_state()
        self.group.set_joint_value_target(joints)
        plan = self._plan_value(self.group.plan())
        if not self.group.execute(plan, wait=True):
            raise AdapterError('{} execution failed'.format(label))
        self.group.stop()

    def verify(self, snapshot, joint_tolerance, translation_tolerance,
               rotation_tolerance):
        current = self.snapshot()
        if max(abs(a-b) for a, b in zip(current['joints'], snapshot['joints'])) > joint_tolerance:
            raise AdapterError('joint return tolerance exceeded')
        if np.linalg.norm(current['tcp'][:3, 3] - snapshot['tcp'][:3, 3]) > translation_tolerance:
            raise AdapterError('TCP translation return tolerance exceeded')
        from .pose_manager import rotation_distance
        if rotation_distance(current['tcp'], snapshot['tcp']) > rotation_tolerance:
            raise AdapterError('TCP rotation return tolerance exceeded')

    def verify_pose(self, target, translation_tolerance, rotation_tolerance):
        current = self.snapshot()['tcp']
        if np.linalg.norm(current[:3, 3] - target[:3, 3]) > translation_tolerance:
            raise AdapterError('target TCP translation tolerance exceeded')
        from .pose_manager import rotation_distance
        if rotation_distance(current, target) > rotation_tolerance:
            raise AdapterError('target TCP rotation tolerance exceeded')

    def attach(self, slot_id, object_pose, object_config):
        size = object_config.get('collision_size_m', [0, 0, 0])
        if len(size) != 3 or any(float(v) <= 0 for v in size):
            raise AdapterError('validated object collision geometry is required')
        name = 'qr_task_object_{}'.format(slot_id)
        self.scene.add_box(name, self._pose(object_pose), size=tuple(size))
        touch_links = object_config.get('touch_links', [self.tcp_link])
        self.scene.attach_box(self.tcp_link, name, touch_links=touch_links)

    def add_world_object(self, slot_id, object_pose, object_config):
        size = object_config.get('collision_size_m', [0, 0, 0])
        if len(size) != 3 or any(float(value) <= 0 for value in size):
            raise AdapterError('validated object collision geometry is required')
        name = 'qr_task_object_{}'.format(slot_id)
        self.scene.add_box(name, self._pose(object_pose), size=tuple(size))

    @staticmethod
    def _set_acm_pair(matrix, first, second, allow):
        from moveit_msgs.msg import AllowedCollisionEntry
        for name in (first, second):
            if name not in matrix.entry_names:
                matrix.entry_names.append(name)
                for row in matrix.entry_values:
                    row.enabled.append(False)
                matrix.entry_values.append(
                    AllowedCollisionEntry(enabled=[False] * len(matrix.entry_names)))
        i = matrix.entry_names.index(first)
        j = matrix.entry_names.index(second)
        matrix.entry_values[i].enabled[j] = bool(allow)
        matrix.entry_values[j].enabled[i] = bool(allow)

    def set_target_contact(self, slot_id, allow, object_config):
        from moveit_msgs.msg import PlanningScene, PlanningSceneComponents
        from moveit_msgs.srv import (ApplyPlanningScene, ApplyPlanningSceneRequest,
                                     GetPlanningScene, GetPlanningSceneRequest)
        get_scene = self.rospy.ServiceProxy('/get_planning_scene', GetPlanningScene)
        apply_scene = self.rospy.ServiceProxy('/apply_planning_scene', ApplyPlanningScene)
        request = GetPlanningSceneRequest()
        request.components.components = PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
        try:
            response = get_scene(request)
        except self.rospy.ServiceException as exc:
            raise AdapterError('cannot read AllowedCollisionMatrix: {}'.format(exc))
        matrix = response.scene.allowed_collision_matrix
        target = 'qr_task_object_{}'.format(slot_id)
        for link in object_config.get('touch_links', [self.tcp_link]):
            self._set_acm_pair(matrix, target, link, allow)
        diff = PlanningScene(is_diff=True, allowed_collision_matrix=matrix)
        try:
            result = apply_scene(ApplyPlanningSceneRequest(scene=diff))
        except self.rospy.ServiceException as exc:
            raise AdapterError('cannot update AllowedCollisionMatrix: {}'.format(exc))
        if not result.success:
            raise AdapterError('MoveIt rejected AllowedCollisionMatrix update')

    def detach(self, slot_id, object_pose, object_config):
        name = 'qr_task_object_{}'.format(slot_id)
        self.scene.remove_attached_object(self.tcp_link, name=name)
        self.scene.add_box(name, self._pose(object_pose),
                           size=tuple(object_config['collision_size_m']))

    def stop(self):
        self.group.stop()
