from piper_x_driver.native_client import Arm
from geometry_msgs.msg import Vector3Stamped
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionFK
from piper_x_interfaces.action import MoveJ, MoveP, MoveT, SetGripper

with Arm() as arm:
    arm.run(MoveJ, 'move_j', joint_positions_rad=[0., 0.4, -0.5, 0., 0.1, 0.], velocity_scale=0.2)
    client = arm.node.create_client(GetPositionFK, 'compute_fk')
    if not client.wait_for_service(timeout_sec=5):
        raise RuntimeError('FK unavailable')
    request = GetPositionFK.Request()
    request.header.frame_id = 'base_link'
    request.fk_link_names = ['tcp_link']
    request.robot_state.joint_state.name = [f'joint{index}' for index in range(1, 7)]
    request.robot_state.joint_state.position = [0., 0.45, -0.5, 0., 0.1, 0.]
    response = arm.wait(client.call_async(request), 5)
    if response.error_code.val != 1:
        raise RuntimeError('FK failed')
    arm.node.destroy_client(client)
    arm.run(MoveP, 'move_p', target=response.pose_stamped[0], motion_mode=MoveP.Goal.DIRECT, velocity_scale=0.1)
    delta = Vector3Stamped()
    delta.header.frame_id = 'base_link'
    delta.vector.x, delta.vector.y, delta.vector.z = 0.008, 0.004, 0.018
    arm.run(MoveT, 'move_t', translation=delta, velocity_scale=0.2)
    for width in (0.04, 0.):
        arm.run(SetGripper, 'set_gripper', opening_m=width)
