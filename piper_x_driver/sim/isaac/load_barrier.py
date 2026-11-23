import json
from pathlib import Path

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive


def main():
    barrier = json.loads(Path(__file__).with_name('barrier.json').read_text())
    rclpy.init()
    node = rclpy.create_node('load_demo_barrier')
    try:
        client = node.create_client(ApplyPlanningScene, 'apply_planning_scene')
        if not client.wait_for_service(timeout_sec=30):
            raise RuntimeError('MoveIt planning scene service unavailable')
        request = ApplyPlanningScene.Request()
        request.scene.is_diff = True
        request.scene.robot_state.is_diff = True
        obstacle = CollisionObject(id=barrier['id'], operation=CollisionObject.ADD)
        obstacle.header.frame_id = barrier['frame']
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = barrier['center']
        pose.orientation.w = 1.0
        obstacle.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=barrier['size'])]
        obstacle.primitive_poses = [pose]
        request.scene.world.collision_objects = [obstacle]
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=5)
        if not future.done() or not future.result().success:
            raise RuntimeError('MoveIt did not confirm demo_barrier')
        node.get_logger().info('MoveIt confirmed demo_barrier')
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
