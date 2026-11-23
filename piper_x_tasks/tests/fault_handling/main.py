from geometry_msgs.msg import PoseStamped
from native_client import Arm
from piper_x_interfaces.action import MoveP, SetGripper
from rclpy.action import ActionClient
from std_srvs.srv import SetBool


def expect_failure(call, expected):
    try:
        call()
    except RuntimeError as exc:
        if expected not in str(exc):
            raise
        print(exc, flush=True)
        return
    raise AssertionError('Expected failure')


with Arm() as arm:
    manual = ActionClient(arm.node, SetGripper, '/piper_x/set_gripper')
    assert manual.wait_for_server(timeout_sec=5)
    handle = arm.wait(manual.send_goal_async(SetGripper.Goal(opening_m=0.01)), 5)
    assert arm.wait(handle.get_result_async(), 5).result.result.error_code == 3
    manual.destroy()
    client = arm.node.create_client(SetBool, '/piper_x/set_enabled')
    assert client.wait_for_service(timeout_sec=5)
    try:
        assert arm.wait(client.call_async(SetBool.Request(data=False)), 5).success
        expect_failure(lambda: arm.run(SetGripper, 'set_gripper', opening_m=0.01), 'error_code=4')
    finally:
        assert arm.wait(client.call_async(SetBool.Request(data=True)), 5).success
    expect_failure(lambda: arm.run(SetGripper, 'set_gripper', opening_m=-0.01), 'error_code=1')
    target = PoseStamped()
    target.header.frame_id = 'base_link'
    target.pose.position.x = 20.
    target.pose.orientation.w = 1.
    expect_failure(lambda: arm.run(MoveP, 'move_p', target=target, velocity_scale=0.1,
                                   motion_mode=MoveP.Goal.AVOID_OBSTACLES), 'move_p:')
    arm.run(SetGripper, 'set_gripper', opening_m=0.)
