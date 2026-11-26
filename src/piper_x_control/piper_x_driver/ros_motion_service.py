import math
import time
import threading
from pathlib import Path

import rclpy
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_srvs.srv import SetBool

from .motion import Motion, PlanningError
from piper_x_interfaces.action import MoveC, MoveJ, MoveL, MoveP, MoveT, SetGripper
from piper_x_interfaces.msg import ArmStatus, MotionFeedback, MotionResult
from piper_x_interfaces.action import RunTask
from .task_runner import run_task


class Service(Node):
    def __init__(self, motion):
        super().__init__('piper_x_ros_motion')
        self.motion = motion
        self.marker = Path('.runtime/task-motion-uncertain')
        self.lock = threading.Lock()
        self.enabled = True
        self.owner = self.run_id = self.active = ''
        self.phase = MotionFeedback.IDLE
        self.last = MotionResult()
        self.last_operation_run = ''
        self.group = ReentrantCallbackGroup()
        self.status = self.create_publisher(ArmStatus, '/piper_x/status',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_timer(1 / motion.config['status_rate_hz'], self.publish_status)
        self.create_service(SetBool, '/piper_x/set_enabled', self.enable, callback_group=self.group)
        self.kinds = dict(move_j=MoveJ, move_p=MoveP, move_l=MoveL, move_c=MoveC, move_t=MoveT, set_gripper=SetGripper)
        self.servers = [ActionServer(self, kind, '/piper_x/' + name,
            lambda handle, kind=kind, name=name: self.execute(handle, kind, name),
            cancel_callback=lambda _: CancelResponse.ACCEPT, callback_group=self.group)
            for name, kind in self.kinds.items()]
        self.servers.append(ActionServer(self, RunTask, '/piper_x/run_task', lambda handle: run_task(self, handle),
            cancel_callback=lambda _: CancelResponse.ACCEPT, callback_group=self.group))

    def publish_status(self):
        with self.lock:
            feedback = time.monotonic() - self.motion.received <= self.motion.config['feedback_timeout_s']
            faulted = self.motion.uncertain or (not self.active and self.marker.exists()) or not feedback
            state = ArmStatus.FAULT if faulted else (
                ArmStatus.BUSY if self.owner or self.active else ArmStatus.READY)
            status = ArmStatus(backend='isaac', state=state, enabled=self.enabled,
                feedback_valid=feedback, owner=self.owner, active_action=self.active,
                motion=MotionFeedback(phase=self.phase), last_result=self.last)
            status.header.stamp = self.get_clock().now().to_msg()
            self.status.publish(status)

    def enable(self, request, response):
        with self.lock:
            if self.active or (request.data and self.marker.exists()):
                response.message = 'Active motion or stop state unavailable'
            else:
                self.enabled = request.data
                response.success = True
                response.message = 'Isaac motion permission updated'
        return response

    def execute(self, handle, kind, name, run_id=''):
        result = kind.Result()
        request = handle.request
        with self.lock:
            code = (MotionResult.BUSY if self.active or run_id != self.run_id
                    else MotionResult.FAULT if self.marker.exists() else MotionResult.DISABLED if not self.enabled
                    else MotionResult.OK)
            if code:
                result.result = MotionResult(error_code=code, message='Driver busy, disabled or faulted')
                handle.abort()
                return result
            self.active = name

        def check_cancel():
            if handle.is_cancel_requested:
                raise InterruptedError('Cancellation requested')

        def phase(value):
            self.phase = value
            handle.publish_feedback(kind.Feedback(feedback=MotionFeedback(phase=value)))

        motion = self.motion
        try:
            self.get_logger().info(f'owner={self.owner} action={name} phase=PREFLIGHT')
            phase(MotionFeedback.PREFLIGHT)
            if hasattr(request, 'velocity_scale'):
                request.velocity_scale = request.velocity_scale or motion.config['default_velocity_scale']
                acceleration = request.acceleration_scale or motion.config['default_acceleration_scale']
                if (not 0 < request.velocity_scale <= motion.config['max_velocity_scale']
                        or not 0 < acceleration <= motion.config['max_acceleration_scale']):
                    raise ValueError('Velocity/acceleration scale outside configured range')
                motion.acceleration_scale = acceleration
            if kind is MoveP:
                pose = request.target
                values = [pose.pose.position.x, pose.pose.position.y, pose.pose.position.z]
                quaternion = [getattr(pose.pose.orientation, axis) for axis in 'xyzw']
                if (request.motion_mode not in (MoveP.Goal.DIRECT, MoveP.Goal.AVOID_OBSTACLES) or pose.header.frame_id != 'base_link'
                        or not all(map(math.isfinite, values + quaternion))
                        or abs(math.hypot(*quaternion) - 1) > 0.001 or not 0 < request.velocity_scale <= 1):
                    raise ValueError('Expected AVOID_OBSTACLES, base_link, finite pose, unit quaternion and valid scale')
            with motion.spin_lock:
                motion.check_owner = check_cancel
                motion.report_phase = phase
                motion.ready()
                check_cancel()
                self.marker.write_text(f'{name}: motion outcome pending')
                if kind is MoveP:
                    phase(MotionFeedback.PLANNING)
                    if request.motion_mode == MoveP.Goal.DIRECT:
                        result.position_error_m, result.orientation_error_rad = motion.direct_pose(pose, request.velocity_scale)
                    else:
                        result.position_error_m, result.orientation_error_rad = motion.pose_goal(
                            pose.pose, pose.header.frame_id, request.velocity_scale)
                elif kind is MoveJ:
                    phase(MotionFeedback.EXECUTING)
                    result.max_joint_error_rad = motion.direct_joints(list(request.joint_positions_rad), request.velocity_scale)
                elif kind is SetGripper:
                    phase(MotionFeedback.EXECUTING)
                    result.actual_opening_m, reached, closing = motion.opening(request.opening_m)
                    result.outcome = (result.REACHED_TARGET if reached else
                                      result.BLOCKED_CLOSING if closing else result.BLOCKED_OPENING)
                    self.get_logger().info(f'Gripper outcome={result.outcome} opening={result.actual_opening_m:.4f} m')
                else:
                    phase(MotionFeedback.PLANNING)
                    if kind is MoveT:
                        waypoints = [motion.linear_target(request.translation)]
                    elif kind is MoveL:
                        waypoints = [self.cartesian_pose(request.target)]
                    else:
                        start, via, end = map(self.cartesian_pose, (request.start, request.via, request.end))
                        for point in (via, end):
                            dot = sum(getattr(start.orientation, axis) * getattr(point.orientation, axis) for axis in 'xyzw')
                            if abs(dot) < 0.9998:
                                raise ValueError('Arc orientation must remain fixed')
                        waypoints = motion.arc(start, via, end)
                    result.position_error_m, result.orientation_error_rad = motion.cartesian(waypoints, request.velocity_scale)
            handle.succeed()
            self.get_logger().info(f'action={name} result=OK')
        except Exception as exc:
            code = (MotionResult.STOP_UNCONFIRMED if motion.uncertain else MotionResult.CANCELED if isinstance(exc, InterruptedError)
                    else MotionResult.INVALID_GOAL if isinstance(exc, ValueError)
                    else MotionResult.FEEDBACK_TIMEOUT if '关节反馈中断' in str(exc)
                    else MotionResult.PLANNING_FAILED if isinstance(exc, PlanningError) else MotionResult.EXECUTION_FAILED)
            result.result = MotionResult(error_code=code, message=str(exc))
            self.get_logger().error(f'{name}: {exc}')
            if code == MotionResult.CANCELED and handle.is_cancel_requested:
                handle.canceled()
            else:
                handle.abort()
        finally:
            motion.check_owner = lambda: None
            motion.report_phase = lambda value: None
            if not motion.uncertain:
                self.marker.unlink(missing_ok=True)
            with self.lock:
                self.last = result.result
                self.last_operation_run = run_id
                self.active = ''
                self.phase = MotionFeedback.IDLE
        return result

    @staticmethod
    def cartesian_pose(stamped):
        pose = stamped.pose
        position = [getattr(pose.position, axis) for axis in 'xyz']
        orientation = [getattr(pose.orientation, axis) for axis in 'xyzw']
        if (stamped.header.frame_id != 'base_link' or not all(map(math.isfinite, position + orientation))
                or abs(math.hypot(*orientation) - 1) > 0.001):
            raise ValueError('Expected finite pose and unit quaternion in base_link')
        return pose


def main():
    rclpy.init()
    motion = Motion()
    motion.spin_lock = threading.Lock()
    motion.uncertain = False
    marker = Path('.runtime/task-motion-uncertain')

    def latch(reason):
        motion.uncertain = True
        marker.write_text(reason)

    motion.latch = latch
    motion.ready()
    service = Service(motion)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(service)

    def feedback_loop():
        while rclpy.ok():
            if motion.spin_lock.acquire(timeout=0.05):
                try:
                    rclpy.spin_once(motion, timeout_sec=0.02)
                finally:
                    motion.spin_lock.release()
            time.sleep(0.01)

    threading.Thread(target=feedback_loop, daemon=True).start()
    service.get_logger().info('INFO: ROS 2 task actions ready')
    executor.spin()


if __name__ == '__main__':
    main()
