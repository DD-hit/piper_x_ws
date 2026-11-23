import argparse
import math
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--timeout', required=True, type=float)
    parser.add_argument('--watch', action='store_true')
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node('piper_x_isaac_feedback_check')
    required = {f'joint{index}' for index in range(1, 7)} | {
        'gripper_joint1', 'gripper_joint2'
    }
    last_feedback = time.monotonic()
    last_stamp = None

    def on_feedback(message):
        nonlocal last_feedback, last_stamp
        positions = dict(zip(message.name, message.position))
        stamp = (message.header.stamp.sec, message.header.stamp.nanosec)
        if stamp != (0, 0) and stamp != last_stamp and all(
            name in positions and math.isfinite(positions[name]) for name in required
        ):
            last_stamp = stamp
            last_feedback = time.monotonic()

    node.create_subscription(JointState, '/piper_x/isaac_joint_states', on_feedback,
                             qos_profile_sensor_data)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
            if last_stamp is not None and not args.watch:
                return 0
            if time.monotonic() - last_feedback >= args.timeout:
                node.get_logger().error(
                    f'No advancing, complete Isaac feedback for {args.timeout:g} s '
                    'on /piper_x/isaac_joint_states. Check simulator process and timeline.'
                )
                return 1
        return 1
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        return 1
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
