#!/usr/bin/env python3
from pathlib import Path
import argparse
import yaml
import rclpy
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

def main():
    p=argparse.ArgumentParser(); p.add_argument('--config',required=True); a, ros_args = p.parse_known_args()
    c=yaml.safe_load(Path(a.config).read_text())
    if c.get('transform_convention') != 'parent_T_child': raise ValueError('expected parent_T_child')
    t=c['translation_m']; q=c['quaternion_xyzw']
    if len(t)!=3 or len(q)!=4: raise ValueError('invalid transform length')
    rclpy.init(args=ros_args); n=rclpy.create_node('piper_x_handeye_static_tf'); m=TransformStamped()
    m.header.stamp=n.get_clock().now().to_msg(); m.header.frame_id=c['parent_frame']; m.child_frame_id=c['child_frame']
    m.transform.translation.x=float(t[0]); m.transform.translation.y=float(t[1]); m.transform.translation.z=float(t[2])
    m.transform.rotation.x=float(q[0]); m.transform.rotation.y=float(q[1]); m.transform.rotation.z=float(q[2]); m.transform.rotation.w=float(q[3])
    try:
        broadcaster = StaticTransformBroadcaster(n)
        broadcaster.sendTransform(m)
        n.get_logger().info(f"Published {m.header.frame_id} -> {m.child_frame_id}")
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.try_shutdown()
if __name__=='__main__': main()
