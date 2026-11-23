import math
import sys
import time
import os
from pathlib import Path
import yaml
from xml.etree import ElementTree

import numpy as np
import rclpy
from scipy.spatial.transform import Rotation
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Point, Pose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, OrientationConstraint, PositionConstraint, PlanningSceneComponents
from moveit_msgs.srv import GetCartesianPath, GetPlanningScene, GetStateValidity, GetPositionIK
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectoryPoint


class Motion(Node):
    def __init__(self):
        super().__init__('piper_x_motion_service')
        config_path = Path(os.environ.get('PIPER_DRIVER_CONFIG', Path(__file__).resolve().parents[1] / 'config/driver.yaml'))
        self.config = yaml.safe_load(config_path.read_text())
        for key, value in self.config.items():
            if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value) or value < 0:
                raise ValueError(f'Invalid driver configuration {key}')
        for key in ('default_velocity_scale', 'default_acceleration_scale', 'max_velocity_scale', 'max_acceleration_scale'):
            if not 0 < self.config[key] <= 1:
                raise ValueError(f'Invalid scale {key}')
        if self.config['status_rate_hz'] <= 0 or self.config['cartesian_step_m'] <= 0:
            raise ValueError('Positive status rate and path step required')
        limits_path = Path(__file__).resolve().parents[1] / 'src/agx_arm_ros/src/agx_arm_moveit/config/joint_limits.yaml'
        self.model_limits = yaml.safe_load(limits_path.read_text())['joint_limits']
        self.acceleration_scale = self.config['default_acceleration_scale']
        self.get_logger().info(f'Driver configuration: {config_path}')
        self.check_owner = lambda: None
        self.report_phase = lambda value: None
        self.ik = self.create_client(GetPositionIK, 'compute_ik')
        self.arm = ActionClient(self, MoveGroup, 'move_action')
        self.trajectory = ActionClient(self, FollowJointTrajectory, 'arm_controller/follow_joint_trajectory')
        self.cartesian_path = self.create_client(GetCartesianPath, 'compute_cartesian_path')
        self.gripper = ActionClient(self, FollowJointTrajectory, 'gripper_controller/follow_joint_trajectory')
        self.joints = [f'joint{index}' for index in range(1, 7)]
        self.state = {}
        self.received = 0.0
        self.stamp = None
        self.create_subscription(JointState, 'joint_states', self.feedback, qos_profile_sensor_data)
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.step = '等待接口和反馈'

    def feedback(self, msg):
        positions = dict(zip(msg.name, msg.position))
        stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        if stamp != self.stamp and all(math.isfinite(positions.get(name, math.nan)) for name in [*self.joints, 'gripper']):
            self.state = positions
            self.received = time.monotonic()
            self.stamp = stamp

    def until(self, condition, seconds, monitor=True, check_owner=True):
        deadline = time.monotonic() + seconds
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if check_owner:
                self.check_owner()
            if monitor and time.monotonic() - self.received > self.config['feedback_timeout_s']:
                raise RuntimeError('关节反馈中断超过配置期限')
            if condition():
                return
            if time.monotonic() >= deadline:
                raise TimeoutError(f'{seconds} s 内未满足条件')
        raise RuntimeError('ROS 已关闭')

    def action(self, client, goal):
        self.report_phase(2 if client is self.arm else 3)
        future = client.send_goal_async(goal)
        try:
            self.until(future.done, self.config['goal_accept_timeout_s'], check_owner=False)
            handle = future.result()
        except BaseException:
            self.latch('目标接收状态未知')
            raise
        if not handle.accepted:
            raise RuntimeError('目标被拒绝')
        result = handle.get_result_async()
        try:
            if client is self.arm:
                timeout = self.config['planning_timeout_s'] + 35
            else:
                end = goal.trajectory.points[-1].time_from_start
                timeout = end.sec + end.nanosec * 1e-9 + self.config['execution_timeout_margin_s']
            self.until(result.done, timeout)
        except BaseException:
            try:
                self.get_logger().warning('动作异常，正在请求取消')
                self.report_phase(5)
                cancel = handle.cancel_goal_async()
                self.until(cancel.done, self.config['cancel_ack_timeout_s'], monitor=False, check_owner=False)
                self.until(result.done, self.config['stop_confirm_timeout_s'], monitor=False, check_owner=False)
                if result.result().status not in (4, 5, 6):
                    raise RuntimeError('未知动作终态')
            except BaseException:
                self.latch('无法确认运动停止')
                raise
            self.get_logger().warning(f'取消后动作终态={result.result().status}')
            raise
        response = result.result()
        code = response.result.error_code
        ok = code.val == 1 if client is self.arm else code == 0
        if response.status != GoalStatus.STATUS_SUCCEEDED or not ok:
            if client is self.arm and code.val in (-1, -2, -3, -6, -10, -11, -12, -13, -14, -15, -16, -17, -18, -19, -31):
                raise PlanningError(f'MoveIt planning error {code.val}')
            raise RuntimeError(f'动作终态={response.status}，错误码={code}')

    def move(self, constraints, scale=0.1):
        goal = MoveGroup.Goal()
        request = goal.request
        request.group_name = 'arm'
        request.pipeline_id = 'ompl'
        request.start_state.is_diff = True
        request.goal_constraints = [constraints]
        request.allowed_planning_time = self.config['planning_timeout_s']
        request.num_planning_attempts = int(self.config['planning_attempts'])
        request.max_velocity_scaling_factor = scale
        request.max_acceleration_scaling_factor = self.acceleration_scale
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True
        self.action(self.arm, goal)

    def joint_pose(self, values):
        if len(values) != 6 or not all(math.isfinite(value) for value in values):
            raise ValueError('需要六个有限关节角')
        target = dict(zip(self.joints, values))
        constraints = Constraints()
        constraints.joint_constraints = [JointConstraint(joint_name=name, position=value,
            tolerance_above=0.003, tolerance_below=0.003, weight=1.) for name, value in target.items()]
        self.move(constraints)
        error = lambda: max(abs(self.state[name] - value) for name, value in target.items())
        self.until(lambda: error() < 0.03, 3)
        self.get_logger().info(f'最大关节误差 {error():.5f} rad')

    def direct_joints(self, values, scale):
        if not 0 < scale <= 1 or not all(map(math.isfinite, values)):
            raise ValueError('Invalid joint target or velocity scale')
        model = ElementTree.parse('.runtime/current_model/piper_x.urdf')
        joint_limits = [model.find(f"joint[@name='{name}']/limit") for name in self.joints]
        limits = [(float(limit.attrib['lower']), float(limit.attrib['upper'])) for limit in joint_limits]
        if any(not lower <= value <= upper for value, (lower, upper) in zip(values, limits)):
            raise ValueError('Joint target outside configured limits')
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.joints
        point = JointTrajectoryPoint(positions=values)
        duration = max(1.5 * abs(self.state[name] - value) / (scale * float(limit.attrib['velocity']))
                       for name, value, limit in zip(self.joints, values, joint_limits))
        duration = max(duration, max(math.sqrt(6 * abs(self.state[name] - value) /
                       (self.acceleration_scale * self.model_limits[name]['max_acceleration']))
                       for name, value in zip(self.joints, values)))
        if duration > 25:
            raise ValueError('Requested motion exceeds the simulation time budget')
        nanoseconds = int(max(1., duration) * 1e9)
        point.time_from_start.sec, point.time_from_start.nanosec = divmod(nanoseconds, 10**9)
        point.velocities = [0.] * 6
        start = JointTrajectoryPoint(positions=[self.state[name] for name in self.joints], velocities=[0.] * 6)
        goal.trajectory.points = [start, point]
        self.action(self.trajectory, goal)
        error = lambda: max(abs(self.state[name] - value) for name, value in zip(self.joints, values))
        self.until(lambda: error() < self.config['joint_tolerance_rad'], self.config['settle_timeout_s'])
        return error()

    def direct_pose(self, target, scale):
        self.until(self.ik.service_is_ready, 5)
        request = GetPositionIK.Request()
        request.ik_request.group_name = 'arm'
        request.ik_request.ik_link_name = 'tcp_link'
        request.ik_request.pose_stamped = target
        request.ik_request.robot_state.is_diff = True
        request.ik_request.robot_state.joint_state.name = self.joints
        request.ik_request.robot_state.joint_state.position = [self.state[name] for name in self.joints]
        request.ik_request.avoid_collisions = False
        request.ik_request.timeout.sec = 2
        future = self.ik.call_async(request)
        self.until(future.done, 5)
        response = future.result()
        if response.error_code.val != 1:
            raise PlanningError(f'IK failed: {response.error_code.val}')
        joints = dict(zip(response.solution.joint_state.name, response.solution.joint_state.position))
        self.direct_joints([joints[name] for name in self.joints], scale)
        return self.verify_pose(target.pose, target.header.frame_id)

    def collision(self, values, object_id):
        scene = self.scene_client
        valid = self.valid_client
        self.until(lambda: scene.service_is_ready() and valid.service_is_ready(), 5)
        request = GetPlanningScene.Request()
        request.components.components = PlanningSceneComponents.WORLD_OBJECT_NAMES
        future = scene.call_async(request)
        self.until(future.done, 5)
        if object_id not in [obj.id for obj in future.result().scene.world.collision_objects]:
            raise RuntimeError(f'规划场景缺少 {object_id}')
        request = GetStateValidity.Request()
        request.group_name = 'arm'
        request.robot_state.is_diff = True
        request.robot_state.joint_state.name = self.joints
        request.robot_state.joint_state.position = values
        future = valid.call_async(request)
        self.until(future.done, 5)
        response = future.result()
        return not response.valid and any(object_id in (pair.contact_body_1, pair.contact_body_2) for pair in response.contacts)

    def tcp(self):
        return self.tf.lookup_transform('base_link', 'tcp_link', rclpy.time.Time()).transform

    def shift_tcp(self, dz):
        start = self.tcp()
        target = Pose(position=Point(
            x=start.translation.x, y=start.translation.y, z=start.translation.z + dz), orientation=start.rotation)
        return self.pose_goal(target, 'base_link', 0.1)

    def pose_goal(self, target, frame, scale):
        position = PositionConstraint(link_name='tcp_link', weight=1.)
        position.header.frame_id = frame
        position.constraint_region.primitives = [SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.002])]
        position.constraint_region.primitive_poses = [target]
        orientation = OrientationConstraint(link_name='tcp_link', orientation=target.orientation,
            absolute_x_axis_tolerance=0.02, absolute_y_axis_tolerance=0.02, absolute_z_axis_tolerance=0.02, weight=1.)
        orientation.header.frame_id = frame
        self.move(Constraints(position_constraints=[position], orientation_constraints=[orientation]), scale)
        return self.verify_pose(target, frame)

    def verify_pose(self, target, frame):
        self.report_phase(4)
        def errors():
            actual = self.tf.lookup_transform(frame, 'tcp_link', rclpy.time.Time()).transform
            distance = math.dist([getattr(target.position, axis) for axis in 'xyz'],
                                 [getattr(actual.translation, axis) for axis in 'xyz'])
            dot = sum(getattr(target.orientation, axis) * getattr(actual.rotation, axis) for axis in 'xyzw')
            return distance, 2 * math.acos(min(1., abs(dot)))

        self.until(lambda: all(value < limit for value, limit in zip(errors(),
                   [self.config['tcp_position_tolerance_m'], self.config['tcp_orientation_tolerance_rad']])),
                   self.config['settle_timeout_s'])
        distance, angle = errors()
        self.get_logger().info(f'TCP 位置误差 {distance:.4f} m，姿态误差 {angle:.4f} rad')
        return distance, angle

    def linear_target(self, translation):
        current = self.tcp()
        vector = translation.vector
        delta = np.array([vector.x, vector.y, vector.z])
        if not np.isfinite(delta).all() or translation.header.frame_id not in ('base_link', 'tcp_link'):
            raise ValueError('Expected finite XYZ delta in base_link or tcp_link')
        if translation.header.frame_id == 'tcp_link':
            rotation = current.rotation
            delta = Rotation.from_quat([rotation.x, rotation.y, rotation.z, rotation.w]).apply(delta)
        return Pose(position=Point(x=current.translation.x + delta[0], y=current.translation.y + delta[1],
                                   z=current.translation.z + delta[2]), orientation=current.rotation)

    def arc(self, start, via, end):
        def xyz(pose):
            return np.array([pose.position.x, pose.position.y, pose.position.z])

        current = self.tcp().translation
        origin = xyz(start)
        if np.linalg.norm(origin - [current.x, current.y, current.z]) > 0.005:
            raise ValueError('TCP is not at the arc start')
        first, last = xyz(via) - origin, xyz(end) - origin
        normal = np.cross(first, last)
        norm = np.linalg.norm(normal)
        if norm < 1e-8:
            raise ValueError('Arc points are coincident or collinear')
        center = origin + (np.dot(first, first) * np.cross(last, normal)
                           + np.dot(last, last) * np.cross(normal, first)) / (2 * norm**2)
        radial = origin - center
        tangent = np.cross(normal / norm, radial)
        sweep = math.atan2(np.dot(xyz(end) - center, tangent), np.dot(xyz(end) - center, radial)) % math.tau
        count = math.ceil(np.linalg.norm(radial) * sweep / self.config['cartesian_step_m'])
        if not 1 <= count <= 200:
            raise ValueError('Arc outside supported length')
        points = [center + math.cos(angle) * radial + math.sin(angle) * tangent
                  for angle in np.linspace(0, sweep, count + 1)[1:]]
        return [Pose(position=Point(x=point[0], y=point[1], z=point[2]), orientation=start.orientation)
                for point in points]

    def cartesian(self, waypoints, scale):
        if not 0 < scale <= 1:
            raise ValueError('Expected velocity scale in (0, 1]')
        orientation = self.tcp().rotation
        for pose in waypoints:
            dot = sum(getattr(orientation, axis) * getattr(pose.orientation, axis) for axis in 'xyzw')
            if abs(dot) < 0.9998:
                raise ValueError('Cartesian motion requires fixed TCP orientation')
        self.until(self.cartesian_path.service_is_ready, 5)
        request = GetCartesianPath.Request(group_name='arm', link_name='tcp_link', waypoints=waypoints,
                                           max_step=self.config['cartesian_step_m'], jump_threshold=2., avoid_collisions=True)
        request.header.frame_id = 'base_link'
        request.start_state.is_diff = True
        future = self.cartesian_path.call_async(request)
        self.until(future.done, 10)
        response = future.result()
        trajectory = response.solution.joint_trajectory
        if response.error_code.val != 1 or response.fraction < 0.9999 or not trajectory.points:
            raise PlanningError(f'Cartesian path incomplete: {response.fraction:.3f}')
        previous = -1
        factor = max(1 / scale, 1 / math.sqrt(self.acceleration_scale))
        for point in trajectory.points:
            if len(point.velocities) != len(trajectory.joint_names) or len(point.accelerations) != len(trajectory.joint_names):
                raise PlanningError('Time-parameterized velocities and accelerations required')
            for index, name in enumerate(trajectory.joint_names):
                limits = self.model_limits[name]
                factor = max(factor, abs(point.velocities[index]) / (limits['max_velocity'] * scale),
                             math.sqrt(abs(point.accelerations[index]) /
                                       (limits['max_acceleration'] * self.acceleration_scale)))
        for point in trajectory.points:
            stamp = point.time_from_start
            duration = int((stamp.sec * 10**9 + stamp.nanosec) * factor)
            if duration <= previous or duration > 30 * 10**9:
                raise PlanningError('Invalid Cartesian trajectory timing')
            previous = duration
            stamp.sec, stamp.nanosec = divmod(duration, 10**9)
            point.velocities = [value / factor for value in point.velocities]
            point.accelerations = [value / factor**2 for value in point.accelerations]
        self.action(self.trajectory, FollowJointTrajectory.Goal(trajectory=trajectory))
        return self.verify_pose(waypoints[-1], 'base_link')

    def opening(self, width):
        if not self.config['min_opening_m'] <= width <= self.config['max_opening_m']:
            raise ValueError('夹爪开度范围 0 至 0.1 m')
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ['gripper']
        point = JointTrajectoryPoint(positions=[width])
        point.time_from_start.sec = 2
        goal.trajectory.points = [point]
        self.action(self.gripper, goal)
        self.until(lambda: abs(self.state['gripper'] - width) < self.config['opening_tolerance_m'], self.config['settle_timeout_s'])
        self.get_logger().info(f"夹爪目标 {width:.3f} m，反馈 {self.state['gripper']:.4f} m")
        return self.state['gripper']

    def ready(self):
        self.state.clear()
        self.until(lambda: self.arm.server_is_ready() and self.trajectory.server_is_ready()
                   and self.gripper.server_is_ready() and self.state
                   and self.tf.can_transform('base_link', 'tcp_link', rclpy.time.Time()), self.config['server_wait_timeout_s'], monitor=False)


class PlanningError(RuntimeError):
    pass
