import json
from pathlib import Path
import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive

rclpy.init()
node = rclpy.create_node('load_grasp_scene')
try:
    client = node.create_client(ApplyPlanningScene, 'apply_planning_scene')
    if not client.wait_for_service(timeout_sec=30):
        raise RuntimeError('Planning scene service unavailable')
    request = ApplyPlanningScene.Request()
    request.scene.is_diff = True
    request.scene.robot_state.is_diff = True
    for item in json.loads(Path(__file__).with_name('grasp_scene.json').read_text())['objects']:
        obj = CollisionObject(id=item['id'], operation=CollisionObject.ADD)
        obj.header.frame_id = 'world'
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = item['center']
        pose.orientation.w = 1.
        obj.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=item['size'])]
        obj.primitive_poses = [pose]
        request.scene.world.collision_objects.append(obj)
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=5)
    if not future.done() or not future.result().success:
        raise RuntimeError('Planning scene update failed')
finally:
    node.destroy_node()
    rclpy.try_shutdown()
