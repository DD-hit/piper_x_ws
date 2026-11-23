from piper_client import Arm

with Arm() as arm:
    ready = [0., 0.4, -0.5, 0., 0.1, 0.]
    arm.call('joints', ready)
    arm.call('shift_tcp_z', 0.025)
    arm.call('opening', 0.04)
    arm.call('opening', 0.)
    arm.call('joints', ready)
