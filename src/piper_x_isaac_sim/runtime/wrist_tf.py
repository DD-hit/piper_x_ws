import rclpy
from geometry_msgs.msg import TransformStamped
from scipy.spatial.transform import Rotation
from tf2_ros import StaticTransformBroadcaster


def main():
    rclpy.init()
    node = rclpy.create_node('piper_wrist_camera_tf')
    mount_rotation = Rotation.from_euler('xyz', [0., -1.22, 1.57])
    optical_rotation = Rotation.from_euler('xyz', [-1.5707963267948966, 0., -1.5707963267948966])
    position = mount_rotation.apply([0.0106, 0.0175, 0.0125]) + [0.029, -0.069, 0.022]
    quaternion = (mount_rotation * optical_rotation).as_quat()
    transform = TransformStamped()
    transform.header.frame_id = 'link6'
    transform.child_frame_id = 'piper_wrist_camera_optical_frame'
    transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z = map(float, position)
    (transform.transform.rotation.x, transform.transform.rotation.y,
     transform.transform.rotation.z, transform.transform.rotation.w) = map(float, quaternion)
    broadcaster = StaticTransformBroadcaster(node)
    broadcaster.sendTransform(transform)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
