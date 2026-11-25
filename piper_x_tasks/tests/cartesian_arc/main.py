import copy
import json
import math
import time
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from piper_x_driver.native_client import Arm
from moveit_msgs.srv import GetPositionFK
from piper_x_interfaces.action import MoveC, MoveP, MoveL, MoveT
from scipy.spatial.transform import Rotation
from tf2_ros import Buffer, TransformListener


def xyz(transform):
    return np.array([getattr(transform.translation, axis) for axis in 'xyz'])


def pose(position, orientation):
    target = PoseStamped()
    target.header.frame_id = 'base_link'
    target.pose.position.x, target.pose.position.y, target.pose.position.z = map(float, position)
    target.pose.orientation = copy.deepcopy(orientation)
    return target


with Arm() as arm:
    buffer = Buffer()
    listener = TransformListener(buffer, arm.node)
    samples = []
    active = [False]

    def capture():
        if active[0] and buffer.can_transform('base_link', 'tcp_link', rclpy.time.Time()):
            transform = buffer.lookup_transform('base_link', 'tcp_link', rclpy.time.Time())
            stamp = transform.header.stamp.sec + transform.header.stamp.nanosec * 1e-9
            if not samples or stamp > samples[-1][0]:
                samples.append((stamp, xyz(transform.transform), transform.transform.rotation))

    arm.node.create_timer(0.02, capture)
    client = arm.node.create_client(GetPositionFK, 'compute_fk')
    if not client.wait_for_service(timeout_sec=5):
        raise RuntimeError('FK service unavailable')
    request = GetPositionFK.Request()
    request.header.frame_id = 'base_link'
    request.fk_link_names = ['tcp_link']
    request.robot_state.joint_state.name = [f'joint{index}' for index in range(1, 7)]
    request.robot_state.joint_state.position = [0., 0.4, -0.5, 0., 0.1, 0.]
    response = arm.wait(client.call_async(request), 5)
    if response.error_code.val != 1:
        raise RuntimeError('Preflight FK failed')
    arm.node.destroy_client(client)
    arm.run(MoveP, 'move_p', target=response.pose_stamped[0],
            motion_mode=MoveP.Goal.AVOID_OBSTACLES, velocity_scale=0.1)
    deadline = time.monotonic() + 5
    while not buffer.can_transform('base_link', 'tcp_link', rclpy.time.Time()):
        rclpy.spin_once(arm.node, timeout_sec=0.1)
        if time.monotonic() > deadline:
            raise RuntimeError('TCP transform unavailable')
    reports = []
    mode = Path(__file__).parent.name
    if mode == 'cartesian_arc':
        current = buffer.lookup_transform('base_link', 'tcp_link', rclpy.time.Time()).transform
        start = xyz(current)
        radius = 0.015
        center = start + np.array([0., 0., radius])
        via = center + np.array([0., radius, 0.])
        end = center + np.array([0., 0., radius])
        samples.clear()
        active[0] = True
        result = arm.run(MoveC, 'move_c', start=pose(start, current.rotation),
            via=pose(via, current.rotation), end=pose(end, current.rotation), velocity_scale=0.2)
        active[0] = False
        points = np.array([point for _, point, _ in samples])
        angles = np.linspace(0, math.pi, 501)
        reference = center + radius * np.column_stack((np.zeros_like(angles), np.sin(angles), -np.cos(angles)))
        distances = np.linalg.norm(points[:, None, :] - reference, axis=2)
        error = max(float(distances.min(axis=1).max()), float(distances.min(axis=0).max()))
        angle_error = max(2 * math.acos(min(1., abs(sum(getattr(rotation, axis) * getattr(current.rotation, axis)
                          for axis in 'xyzw')))) for _, _, rotation in samples)
        reports.append(dict(mode='arc', samples=len(samples), path_error_m=error,
                            max_orientation_error_rad=angle_error, endpoint_error_m=result.position_error_m))
    else:
        for frame in ('absolute', 'base_link', 'tcp_link'):
            current = buffer.lookup_transform('base_link', 'tcp_link', rclpy.time.Time()).transform
            start = xyz(current)
            delta = np.array([0.008, 0.004, 0.012])
            direction = delta
            if frame == 'tcp_link':
                direction = Rotation.from_quat([getattr(current.rotation, axis) for axis in 'xyzw']).apply(delta)
            target = start + direction
            samples.clear()
            active[0] = True
            if frame == 'absolute':
                result = arm.run(MoveL, 'move_l', target=pose(target, current.rotation), velocity_scale=0.2)
            else:
                translation = Vector3Stamped()
                translation.header.frame_id = frame
                translation.vector.x, translation.vector.y, translation.vector.z = map(float, delta)
                result = arm.run(MoveT, 'move_t', translation=translation, velocity_scale=0.2)
            active[0] = False
            points = np.array([point for _, point, _ in samples])
            ratio = np.clip((points - start) @ direction / np.dot(direction, direction), 0., 1.)
            errors = np.linalg.norm(points - (start + ratio[:, None] * direction), axis=1)
            error = float(np.max(errors))
            angle_error = max(2 * math.acos(min(1., abs(sum(getattr(rotation, axis) * getattr(current.rotation, axis)
                              for axis in 'xyzw')))) for _, _, rotation in samples)
            reports.append(dict(mode=frame, samples=len(samples), path_error_m=error,
                                max_orientation_error_rad=angle_error, endpoint_error_m=result.position_error_m))
    Path('logs').mkdir(exist_ok=True)
    Path('logs/path-verification.json').write_text(json.dumps(reports, indent=2))
    for report in reports:
        if (report['samples'] < 5 or report['path_error_m'] > 0.005 or report['endpoint_error_m'] > 0.01
                or report['max_orientation_error_rad'] > 0.06):
            raise RuntimeError(f'Path verification failed: {report}')
    print(json.dumps(reports, indent=2), flush=True)
