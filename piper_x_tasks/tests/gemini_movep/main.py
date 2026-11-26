"""Verify MoveP detours around obstacles that block individual wrist attachments."""
from itertools import product
import os
import time
import copy
import json
import tempfile
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene, GetPositionFK, GetStateValidity
from piper_x_interfaces.action import MoveJ, MoveP
from piper_x_interfaces.msg import MotionResult
from rclpy.action import ActionClient
from scipy.spatial.transform import Rotation
from shape_msgs.msg import SolidPrimitive
from sensor_msgs.msg import JointState
from rclpy.qos import qos_profile_sensor_data


def main():
    rclpy.init()
    node = rclpy.create_node('verify_gemini_movep')
    joints = {}
    samples = []
    names = [f"joint{i}" for i in range(1,7)]
    mirror_file = Path(__file__).resolve().parents[3] / '.runtime/gemini-movep-fixture.json'

    def show_fixture(pose, dimensions, pause):
        p, q = pose.position, pose.orientation
        fixture = dict(position=[p.x,p.y,p.z], quaternion=[q.x,q.y,q.z,q.w],
                       dimensions=dimensions, expires=time.time()+pause+150)
        with tempfile.NamedTemporaryFile(mode='w', dir=mirror_file.parent, delete=False) as out:
            json.dump(fixture,out)
        Path(out.name).replace(mirror_file)

    def feedback(msg):
        joints.update(zip(msg.name,msg.position))
        if all(name in joints for name in names):
            samples.append([joints[name] for name in names])
    node.create_subscription(JointState, "/joint_states", feedback, qos_profile_sensor_data)
    joint_action = ActionClient(node, MoveJ, '/piper_x/move_j')
    action = ActionClient(node, MoveP, '/piper_x/move_p')
    fk = node.create_client(GetPositionFK, '/compute_fk')
    validity = node.create_client(GetStateValidity, '/check_state_validity')
    apply = node.create_client(ApplyPlanningScene, '/apply_planning_scene')

    def wait(future, timeout=10):
        rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
        if not future.done():
            raise TimeoutError('ROS request timed out')
        return future.result()

    def call(client, request):
        return wait(client.call_async(request))

    def configure_obstacle(obstacle, operation):
        obstacle.operation = operation
        scene = PlanningScene(is_diff=True)
        scene.robot_state.is_diff = True
        scene.world.collision_objects = [obstacle]
        assert call(apply, ApplyPlanningScene.Request(scene=scene)).success

    def move(client, goal):
        handle = wait(client.send_goal_async(goal))
        assert handle.accepted, 'MoveP goal rejected before planning'
        try:
            return wait(handle.get_result_async(), 40).result
        except TimeoutError:
            wait(handle.cancel_goal_async())
            raise

    try:
        assert action.wait_for_server(timeout_sec=25) and joint_action.wait_for_server(timeout_sec=10)
        for client in (fk, validity, apply):
            assert client.wait_for_service(timeout_sec=10)
        cases = [
            ('gemini_camera_link', 'gemini_camera_link', .22, [.012,.018,.036], [(-.010,y,0) for y in (0,-.02,-.045,-.07,-.095)]),
            ('gemini_mount_bracket', 'link6', -.60, [.045,.027,.021],
             list(product((-.045,.045),(-.080,-.085,-.075),(.006,.005,.004)))),
        ]
        for expected, frame, angle, dimensions, candidates in cases:
            state = GetStateValidity.Request(group_name='arm')
            state.robot_state.is_diff = True
            state.robot_state.joint_state.name = [f'joint{i}' for i in range(1,7)]
            state.robot_state.joint_state.position = [angle, .35, -.35, 0., 0., 0.]
            assert call(validity,state).valid, f'Unobstructed goal invalid: {expected}'
            request = GetPositionFK.Request(robot_state=state.robot_state)
            request.header.frame_id = 'base_link'
            request.fk_link_names = ['tcp_link',frame]
            response = call(fk,request)
            assert response.error_code.val == 1
            target, reference = response.pose_stamped
            p, q = reference.pose.position, reference.pose.orientation
            rotation = Rotation.from_quat([q.x,q.y,q.z,q.w])
            origin = np.array([p.x,p.y,p.z])
            obstacle = CollisionObject(id='test_gemini_movep_fixture')
            obstacle.header.frame_id = 'base_link'
            obstacle.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=dimensions)]
            current = GetStateValidity.Request()
            current.robot_state.is_diff = True
            current.robot_state.joint_state.name = state.robot_state.joint_state.name
            before = np.array([joints[name] for name in current.robot_state.joint_state.name])
            current.robot_state.joint_state.position = before.tolist()
            start = copy.deepcopy(state)
            finish = copy.deepcopy(state)
            start.robot_state.joint_state.position[0] -= .58
            finish.robot_state.joint_state.position[0] += 1.05
            result = move(joint_action, MoveJ.Goal(joint_positions_rad=start.robot_state.joint_state.position,
                                                  velocity_scale=.30,acceleration_scale=.30))
            assert result.result.error_code == MotionResult.OK, result
            request.fk_link_names=['tcp_link']
            request.robot_state=finish.robot_state
            finish_fk=call(fk,request)
            assert finish_fk.error_code.val==1
            target=finish_fk.pose_stamped[0]
            try:
                for local in candidates:
                    xyz = origin + rotation.apply(local)
                    fixture = Pose()
                    fixture.position.x, fixture.position.y, fixture.position.z = xyz.tolist()
                    fixture.orientation = q
                    obstacle.primitive_poses = [fixture]
                    configure_obstacle(obstacle,CollisionObject.ADD)
                    result = call(validity,state)
                    contacts = {(c.contact_body_1,c.contact_body_2) for c in result.contacts}
                    if (not result.valid and contacts
                            and all(set(pair)=={expected,obstacle.id} for pair in contacts)
                            and call(validity,start).valid and call(validity,finish).valid):
                        print(f'{expected}: start/goal clear; direct joint interpolation hits only {expected}',flush=True)
                        break
                else:
                    raise AssertionError(f'No isolated mid-path contact for {expected}')
                pause = float(os.environ.get('PIPER_GEMINI_TEST_PAUSE','5'))
                show_fixture(fixture, dimensions, pause)
                time.sleep(pause)
                samples.clear()
                goal=MoveP.Goal(motion_mode=MoveP.Goal.AVOID_OBSTACLES,target=target,
                                velocity_scale=.15,acceleration_scale=.15)
                for attempt in range(3):
                    outcome=move(action,goal)
                    if outcome.result.error_code != MotionResult.PLANNING_FAILED:
                        break
                    print(f'{expected}: planner rejected attempt {attempt+1}: {outcome.result.message}',flush=True)
                    assert not samples or np.max(np.ptp(samples,axis=0)) < .02, 'Planning failure moved the arm'

                assert outcome.result.error_code == MotionResult.OK, outcome
                measured=np.array(samples)
                assert len(measured)>5
                checked=np.unique(np.round(measured[::max(1,len(measured)//75)],4),axis=0)
                for q_sample in checked:
                    current.robot_state.joint_state.position=q_sample.tolist()
                    a=call(validity,current)
                    assert a.valid, [(c.contact_body_1,c.contact_body_2) for c in a.contacts]
                print(f'{expected}: MoveP success, TCP error {outcome.position_error_m*1000:.2f} mm, '
                      f'{len(checked)} collision-free feedback samples',flush=True)
            finally:
                mirror_file.unlink(missing_ok=True)
                configure_obstacle(obstacle,CollisionObject.REMOVE)
        print('PASS: MoveP detours around both wrist attachments',flush=True)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
