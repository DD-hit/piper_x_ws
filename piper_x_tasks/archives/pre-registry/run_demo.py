import math
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Point, Pose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, OrientationConstraint, PositionConstraint, PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene, GetStateValidity
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectoryPoint


class Demo(Node):
    def __init__(self):
        super().__init__('piper_x_empty_gripper_demo')
        self.arm = ActionClient(self, MoveGroup, 'move_action')
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

    def until(self, condition, seconds, monitor=True):
        deadline = time.monotonic() + seconds
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if monitor and time.monotonic() - self.received > 2:
                raise RuntimeError('关节反馈中断超过 2 s')
            if condition():
                return
            if time.monotonic() >= deadline:
                raise TimeoutError(f'{seconds} s 内未满足条件')
        raise RuntimeError('ROS 已关闭')

    def action(self, client, goal):
        future = client.send_goal_async(goal)
        try:
            self.until(future.done, 5)
        except BaseException:
            self.get_logger().error('目标接收状态未知，请确认控制器状态后再运行')
            raise
        handle = future.result()
        if not handle.accepted:
            raise RuntimeError('目标被拒绝')
        result = handle.get_result_async()
        try:
            self.until(result.done, 35)
        except BaseException:
            cancel = handle.cancel_goal_async()
            self.until(cancel.done, 3, monitor=False)
            self.until(result.done, 5, monitor=False)
            self.get_logger().warning(f'取消后动作终态={result.result().status}')
            raise
        response = result.result()
        code = response.result.error_code
        ok = code.val == 1 if client is self.arm else code == 0
        if response.status != GoalStatus.STATUS_SUCCEEDED or not ok:
            raise RuntimeError(f'动作终态={response.status}，错误码={code}')

    def move(self, constraints):
        goal = MoveGroup.Goal()
        request = goal.request
        request.group_name = 'arm'
        request.pipeline_id = 'ompl'
        request.start_state.is_diff = True
        request.goal_constraints = [constraints]
        request.allowed_planning_time = 5.0
        request.num_planning_attempts = 3
        request.max_velocity_scaling_factor = 0.1
        request.max_acceleration_scaling_factor = 0.1
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True
        self.action(self.arm, goal)

    def joint_pose(self, values):
        target = dict(zip(self.joints, values))
        constraints = Constraints()
        constraints.joint_constraints = [JointConstraint(joint_name=name, position=value,
            tolerance_above=0.003, tolerance_below=0.003, weight=1.) for name, value in target.items()]
        self.move(constraints)
        error = lambda: max(abs(self.state[name] - value) for name, value in target.items())
        self.until(lambda: error() < 0.03, 3)
        self.get_logger().info(f'最大关节误差 {error():.5f} rad')

    def barrier_trial(self):
        scene = self.create_client(GetPlanningScene, 'get_planning_scene')
        valid = self.create_client(GetStateValidity, 'check_state_validity')
        self.until(lambda: scene.service_is_ready() and valid.service_is_ready(), 5)
        request = GetPlanningScene.Request()
        request.components.components = PlanningSceneComponents.WORLD_OBJECT_NAMES
        future = scene.call_async(request)
        self.until(future.done, 5)
        if 'demo_barrier' not in [obj.id for obj in future.result().scene.world.collision_objects]:
            raise RuntimeError('规划场景缺少 demo_barrier')
        middle = [0.58, 0.9, -0.5, 0., 0.1, 0.]
        request = GetStateValidity.Request()
        request.group_name = 'arm'
        request.robot_state.is_diff = True
        request.robot_state.joint_state.name = self.joints
        request.robot_state.joint_state.position = middle
        future = valid.call_async(request)
        self.until(future.done, 5)
        response = future.result()
        if response.valid or not any('demo_barrier' in (pair.contact_body_1, pair.contact_body_2) for pair in response.contacts):
            raise RuntimeError('未验证关节直插路径中点与挡板冲突')
        for angle in [-0.07, 1.23]:
            self.get_logger().info(f'挡板测试，目标 joint1={angle} rad')
            self.joint_pose([angle, *middle[1:]])

    def tcp(self):
        return self.tf.lookup_transform('base_link', 'tcp_link', rclpy.time.Time()).transform

    def shift_tcp(self):
        start = self.tcp()
        target = Pose(position=Point(
            x=start.translation.x, y=start.translation.y, z=start.translation.z + 0.025), orientation=start.rotation)
        position = PositionConstraint(link_name='tcp_link', weight=1.)
        position.header.frame_id = 'base_link'
        position.constraint_region.primitives = [SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.002])]
        position.constraint_region.primitive_poses = [target]
        orientation = OrientationConstraint(link_name='tcp_link', orientation=target.orientation,
            absolute_x_axis_tolerance=0.02, absolute_y_axis_tolerance=0.02, absolute_z_axis_tolerance=0.02, weight=1.)
        orientation.header.frame_id = 'base_link'
        self.move(Constraints(position_constraints=[position], orientation_constraints=[orientation]))

        def errors():
            actual = self.tcp()
            distance = math.dist([getattr(target.position, axis) for axis in 'xyz'],
                                 [getattr(actual.translation, axis) for axis in 'xyz'])
            dot = sum(getattr(target.orientation, axis) * getattr(actual.rotation, axis) for axis in 'xyzw')
            return distance, 2 * math.acos(min(1., abs(dot)))

        self.until(lambda: all(value < limit for value, limit in zip(errors(), [0.01, 0.06])), 3)
        distance, angle = errors()
        self.get_logger().info(f'TCP 位置误差 {distance:.4f} m，姿态误差 {angle:.4f} rad')

    def opening(self, width):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ['gripper']
        point = JointTrajectoryPoint(positions=[width])
        point.time_from_start.sec = 2
        goal.trajectory.points = [point]
        self.action(self.gripper, goal)
        self.until(lambda: abs(self.state['gripper'] - width) < 0.003, 3)
        self.get_logger().info(f"夹爪目标 {width:.3f} m，反馈 {self.state['gripper']:.4f} m")

    def run(self):
        self.until(lambda: self.arm.server_is_ready() and self.gripper.server_is_ready() and self.state
                   and self.tf.can_transform('base_link', 'tcp_link', rclpy.time.Time()), 15, monitor=False)
        ready = lambda: self.joint_pose([0., 0.4, -0.5, 0., 0.1, 0.])
        steps = [('预备姿态', ready), ('TCP 上移 25 mm', self.shift_tcp),
                                     ('张开夹爪', lambda: self.opening(0.04)), ('闭合夹爪', lambda: self.opening(0.)),
                 ]
        if self.declare_parameter('barrier_trial', False).value:
            steps.append(('绕过挡板', self.barrier_trial))
        steps.append(('返回预备姿态', ready))
        for self.step, operation in steps:
            self.get_logger().info(self.step)
            operation()


if __name__ == '__main__':
    rclpy.init()
    demo = Demo()
    try:
        demo.run()
    except (Exception, KeyboardInterrupt) as exc:
        demo.get_logger().error(f'任务终止，步骤={demo.step}，原因={exc!r}')
        sys.exit(1)
    finally:
        demo.destroy_node()
        rclpy.try_shutdown()
