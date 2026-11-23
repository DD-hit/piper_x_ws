"""Verify the official mock backend only: plan, execute and check joint feedback."""
import json
import time
from pathlib import Path
import rclpy
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetMotionPlan
from moveit_msgs.msg import Constraints, JointConstraint
from control_msgs.action import FollowJointTrajectory

import argparse
parser=argparse.ArgumentParser()
parser.add_argument('--backend',choices=['mock','isaac'],default='mock')
args=parser.parse_args()
rclpy.init()
node = rclpy.create_node('piper_x_mock_verification')
latest = {}
sub = node.create_subscription(JointState, ('/piper_x/physics_joint_states' if args.backend=='isaac' else '/control/joint_states'),
    lambda msg: latest.update(zip(msg.name, msg.position)), 10)

def wait(future, timeout):
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    if not future.done():
        raise TimeoutError('ROS request timed out')
    return future.result()

try:
    client = node.create_client(GetMotionPlan, '/plan_kinematic_path')
    if not client.wait_for_service(timeout_sec=30):
        raise RuntimeError('MoveIt planning service unavailable')
    deadline = time.monotonic() + 15
    while not all(f'joint{i}' in latest for i in range(1, 7)):
        if time.monotonic() > deadline:
            raise RuntimeError('No six-joint mock feedback')
        rclpy.spin_once(node, timeout_sec=0.2)
    target = [0.0, 0.4, -0.5, 0.0, 0.1, 0.0]
    request = GetMotionPlan.Request()
    m = request.motion_plan_request
    m.group_name = 'arm'
    m.pipeline_id = 'ompl'
    m.allowed_planning_time = 10.0
    m.num_planning_attempts = 5
    m.max_velocity_scaling_factor = 0.1
    m.max_acceleration_scaling_factor = 0.1
    m.start_state.joint_state.name = [f'joint{i}' for i in range(1, 7)]
    m.start_state.joint_state.position = [latest[n] for n in m.start_state.joint_state.name]
    m.start_state.is_diff = True
    c = Constraints()
    for i, q in enumerate(target, 1):
        c.joint_constraints.append(JointConstraint(joint_name=f'joint{i}', position=q,
            tolerance_above=0.001, tolerance_below=0.001, weight=1.0))
    m.goal_constraints = [c]
    response = wait(client.call_async(request), 30).motion_plan_response
    if response.error_code.val != 1:
        raise RuntimeError(f'Planning failed: {response.error_code.val}')
    trajectory = response.trajectory.joint_trajectory
    if not trajectory.points:
        raise RuntimeError('Empty trajectory')
    action = ActionClient(node, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
    if not action.wait_for_server(timeout_sec=15):
        raise RuntimeError('Trajectory action unavailable')
    goal = FollowJointTrajectory.Goal()
    goal.trajectory = trajectory
    handle = wait(action.send_goal_async(goal), 10)
    if not handle.accepted:
        raise RuntimeError('Trajectory rejected')
    result = wait(handle.get_result_async(), 60)
    if result.status != 4 or result.result.error_code != 0:
        raise RuntimeError(f'Execution failed: {result.status}, {result.result}')
    deadline = time.monotonic() + 2
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    expected = dict(zip(trajectory.joint_names, trajectory.points[-1].positions))
    error = max(abs(latest[n] - q) for n, q in expected.items())
    if error > (0.03 if args.backend=='isaac' else 0.01):
        raise RuntimeError(f'Final mock joint error {error} rad')
    report = {'backend': args.backend, 'arm_type': 'piper_x', 'planning': 'PASS',
        'execution': 'PASS', 'trajectory_points': len(trajectory.points),
        'final_max_error_rad': error, 'final_joint_positions': latest}
    Path(f'.runtime/{args.backend}_verification.json').write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2), flush=True)
finally:
    node.destroy_node()
    rclpy.shutdown()
