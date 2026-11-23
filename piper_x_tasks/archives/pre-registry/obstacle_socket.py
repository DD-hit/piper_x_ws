from piper_client import Arm

with Arm() as arm:
    middle = [0.58, 0.9, -0.5, 0., 0.1, 0.]
    if not arm.call('collision', middle, 'demo_barrier'):
        raise RuntimeError('测试前提不成立：插值中点与挡板没有碰撞')
    for angle in [-0.07, 1.23]:
        arm.call('joints', [angle, *middle[1:]])
    arm.call('joints', [0., 0.4, -0.5, 0., 0.1, 0.])
