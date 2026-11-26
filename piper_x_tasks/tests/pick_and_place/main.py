import json
import time
from pathlib import Path

import os
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject, PlanningSceneComponents
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from piper_x_driver.native_client import Arm
from piper_x_interfaces.action import MoveL, MoveP, SetGripper
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

root = Path(__file__).resolve().parents[3]
reports = []
scene_path = root / 'src/piper_x_isaac_sim/runtime/grasp_scene.json'
scene_objects = json.loads(scene_path.read_text())['objects']
initial_cube_x, initial_cube_y, _ = next(
    obj['center'] for obj in scene_objects if obj['id'] == 'grasp_cube'
)
pick_x = initial_cube_x


def telemetry():
    value = json.loads((root / '.runtime/grasp-state.json').read_text())
    if time.time() - value['wall_time'] > 2:
        raise RuntimeError('Isaac object telemetry stale')
    return value


def state():
    return telemetry()['position']


def snapshot(stage):
    sample = telemetry()
    reports.append(dict(sample, stage=stage))
    Path(f"logs/{os.environ['PIPER_RUN_ID']}-grasp.json").write_text(json.dumps(reports, indent=2))


def target(y, z):
    pose = PoseStamped()
    pose.header.frame_id = 'base_link'
    pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = pick_x, y, z
    pose.pose.orientation.x = 1.
    pose.pose.orientation.w = 0.
    return pose


with Arm() as arm:
    pick_y = initial_cube_y
    if "locate_pick_target" in globals():
        pick_x, pick_y = locate_pick_target(arm)
    transforms = Buffer()
    listener = TransformListener(transforms, arm.node)

    def transfer_target(y, z):
        pose = target(y, z)
        pose.pose.orientation = transforms.lookup_transform('base_link', 'tcp_link', Time()).transform.rotation
        return pose
    apply = arm.node.create_client(ApplyPlanningScene, 'apply_planning_scene')
    get = arm.node.create_client(GetPlanningScene, 'get_planning_scene')
    assert apply.wait_for_service(timeout_sec=5) and get.wait_for_service(timeout_sec=5)

    def update(scene):
        scene.is_diff = True
        scene.robot_state.is_diff = True
        if not arm.wait(apply.call_async(ApplyPlanningScene.Request(scene=scene)), 5).success:
            raise RuntimeError('Scene update failed')

    request = GetPlanningScene.Request()
    request.components.components = PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
    scene = arm.wait(get.call_async(request), 5).scene
    matrix = scene.allowed_collision_matrix
    if 'grasp_cube' not in matrix.default_entry_names:
        matrix.default_entry_names.append('grasp_cube')
        matrix.default_entry_values.append(False)
    from moveit_msgs.msg import AllowedCollisionEntry
    names = matrix.entry_names
    if 'grasp_cube' not in names:
        for entry in matrix.entry_values:
            entry.enabled.append(False)
        names.append('grasp_cube')
        matrix.entry_values.append(AllowedCollisionEntry(enabled=[False] * len(names)))
    index = names.index('grasp_cube')
    for name in ['gripper_link1', 'gripper_link2']:
        if name not in names:
            for entry in matrix.entry_values:
                entry.enabled.append(False)
            names.append(name)
            matrix.entry_values.append(AllowedCollisionEntry(enabled=[False] * len(names)))
        other = names.index(name)
        matrix.entry_values[index].enabled[other] = True
        matrix.entry_values[other].enabled[index] = True
    update(scene)
    start = state()
    if abs(start[0] - initial_cube_x) > .01 or abs(start[1] - initial_cube_y) > .01:
        raise RuntimeError('Reset scene before each grasp attempt')
    opened = arm.run(SetGripper, 'set_gripper', opening_m=.08)
    if opened.outcome != opened.REACHED_TARGET:
        raise RuntimeError('Gripper opening obstructed before approach')
    arm.run(MoveP, 'move_p', target=target(pick_y, .28), motion_mode=MoveP.Goal.AVOID_OBSTACLES,
            velocity_scale=.1, acceleration_scale=.1)
    arm.run(MoveL, 'move_l', target=target(pick_y, .235), velocity_scale=.1, acceleration_scale=.1)
    snapshot('before_close')
    grip = arm.run(SetGripper, 'set_gripper', opening_m=0.)
    if grip.outcome != grip.BLOCKED_CLOSING:
        raise RuntimeError('Closing did not encounter the expected obstruction')
    snapshot('closed')
    time.sleep(2)
    snapshot('closed_hold')
    arm.run(MoveL, 'move_l', target=target(pick_y, .28), velocity_scale=.1, acceleration_scale=.1)
    time.sleep(.3)
    lifted = state()
    snapshot('lift')
    time.sleep(2)
    snapshot('lift_hold')
    lifted = state()
    if lifted[2] < start[2] + .035:
        raise RuntimeError(f'Physical lift failed: {start} -> {lifted}')
    request.components.components = PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
    scene = arm.wait(get.call_async(request), 5).scene
    cube = next(obj for obj in scene.world.collision_objects if obj.id == 'grasp_cube')
    from geometry_msgs.msg import Quaternion
    measured = telemetry()['orientation']
    cube.primitive_poses[0].orientation = Quaternion(x=measured[1], y=measured[2], z=measured[3], w=measured[0])
    for pose in cube.primitive_poses:
        pose.position.x, pose.position.y, pose.position.z = lifted
    scene.world.collision_objects = []
    scene.robot_state.attached_collision_objects = [AttachedCollisionObject(link_name='tcp_link', object=cube,
        touch_links=['gripper_link1', 'gripper_link2'])]
    update(scene)
    arm.run(MoveP, 'move_p', target=target(.08, .28), motion_mode=MoveP.Goal.AVOID_OBSTACLES,
            velocity_scale=.1, acceleration_scale=.05)
    carried = state()
    snapshot('after_transfer')
    if abs(carried[1] - .08) > .02 or carried[2] < start[2] + .025:
        raise RuntimeError(f'Object dropped during transport: {carried}')
    arm.run(MoveL, 'move_l', target=transfer_target(.08, .238), velocity_scale=.1, acceleration_scale=.1)
    snapshot('before_release')
    opened = arm.run(SetGripper, 'set_gripper', opening_m=.08)
    if opened.outcome != opened.REACHED_TARGET:
        raise RuntimeError('Gripper opening obstructed during release')
    time.sleep(.5)
    placed = state()
    snapshot('after_release')
    scene.world.collision_objects = []
    scene.robot_state.attached_collision_objects = [AttachedCollisionObject(link_name='tcp_link',
        object=CollisionObject(id='grasp_cube', operation=CollisionObject.REMOVE))]
    cube.primitive_poses[0].position.x, cube.primitive_poses[0].position.y, cube.primitive_poses[0].position.z = placed
    scene.world.collision_objects = [cube]
    update(scene)
    arm.run(MoveL, 'move_l', target=transfer_target(.08, .28), velocity_scale=.1, acceleration_scale=.1)
    placed = state()
    snapshot('placed')
    if abs(placed[0] - .35) > .025 or abs(placed[1] - .08) > .025 or abs(placed[2] - .096) > .015:
        raise RuntimeError(f'Placement failed: {placed}')
    print(json.dumps(reports, indent=2), flush=True)
