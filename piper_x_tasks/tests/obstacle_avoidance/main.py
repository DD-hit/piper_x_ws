from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionFK, GetStateValidity
from piper_x_driver.native_client import Arm
from piper_x_interfaces.action import MoveP


def query(arm, kind, name, request):
    client = arm.node.create_client(kind, name)
    try:
        if not client.wait_for_service(timeout_sec=5):
            raise RuntimeError(f'{name} unavailable')
        return arm.wait(client.call_async(request), 5)
    finally:
        arm.node.destroy_client(client)


def target(arm, angle):
    request = GetPositionFK.Request()
    request.header.frame_id = 'base_link'
    request.fk_link_names = ['tcp_link']
    request.robot_state.joint_state.name = [f'joint{index}' for index in range(1, 7)]
    request.robot_state.joint_state.position = [angle, 0.9, -0.5, 0., 0.1, 0.]
    response = query(arm, GetPositionFK, 'compute_fk', request)
    if response.error_code.val != 1:
        raise RuntimeError('Could not compute fixture target')
    return PoseStamped(header=response.pose_stamped[0].header, pose=response.pose_stamped[0].pose)


with Arm() as arm:
    request = GetStateValidity.Request(group_name='arm')
    request.robot_state.is_diff = True
    request.robot_state.joint_state.name = [f'joint{index}' for index in range(1, 7)]
    request.robot_state.joint_state.position = [0.58, 0.9, -0.5, 0., 0.1, 0.]
    response = query(arm, GetStateValidity, 'check_state_validity', request)
    if response.valid or not any('demo_barrier' in (pair.contact_body_1, pair.contact_body_2) for pair in response.contacts):
        raise RuntimeError('挡板测试前提不成立')
    for angle in (-0.07, 1.23):
        arm.run(MoveP, 'move_p', motion_mode=MoveP.Goal.AVOID_OBSTACLES,
                target=target(arm, angle), velocity_scale=0.1)
