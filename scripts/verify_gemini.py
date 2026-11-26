"""Check streaming, optical TF and an obstacle that hits only the camera."""
import time
import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene, GetStateValidity
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from shape_msgs.msg import SolidPrimitive
from tf2_ros import Buffer, TransformListener

rclpy.init()
node=rclpy.create_node('verify_gemini')
tf=Buffer()
listener=TransformListener(tf,node)
msgs={}
for stream,image in [('color','image_raw'),('depth','image_rect_raw')]:
    for leaf,cls in [(image,Image),('camera_info',CameraInfo)]:
        name=f'{stream}/{leaf}'
        node.create_subscription(cls,'/piper_x/camera/'+name,
                                 lambda msg,name=name: msgs.setdefault(name,msg),qos_profile_sensor_data)
validity=node.create_client(GetStateValidity,'/check_state_validity')
apply=node.create_client(ApplyPlanningScene,'/apply_planning_scene')

def call(client,req):
    f=client.call_async(req)
    rclpy.spin_until_future_complete(node,f,timeout_sec=8)
    assert f.done() and f.result() is not None
    return f.result()

deadline=time.monotonic()+65
while time.monotonic()<deadline:
    rclpy.spin_once(node,timeout_sec=.25)
    if len(msgs)==4 and validity.service_is_ready() and apply.service_is_ready():
        if all(tf.can_transform('link6',msg.header.frame_id,rclpy.time.Time()) for msg in msgs.values()):
            break
assert len(msgs)==4, list(msgs)
for name,msg in msgs.items():
    frame=f'gemini_camera_{name.split("/")[0]}_optical_frame'
    assert msg.header.frame_id==frame and tf.can_transform('link6',frame,rclpy.time.Time())
    assert (msg.width,msg.height)==(640,480)
    print(name,msg.header.frame_id,getattr(msg,'encoding','CameraInfo'),flush=True)

request=GetStateValidity.Request()
request.robot_state.is_diff=True
baseline=call(validity,request)
assert baseline.valid,[(p.contact_body_1,p.contact_body_2) for p in baseline.contacts]
print('baseline collision-free',flush=True)
obstacle=CollisionObject()
obstacle.id='gemini_collision_smoke'
obstacle.header.frame_id='gemini_camera_link'
box=SolidPrimitive(type=SolidPrimitive.BOX,dimensions=[.012,.018,.036])
pose=Pose()
pose.position.x=-.010
pose.orientation.w=1.
obstacle.primitives=[box]
obstacle.primitive_poses=[pose]
def update(operation):
    obstacle.operation=operation
    request=ApplyPlanningScene.Request()
    request.scene=PlanningScene(is_diff=True)
    request.scene.robot_state.is_diff=True
    request.scene.world.collision_objects=[obstacle]
    assert call(apply,request).success
try:
    update(CollisionObject.ADD)
    result=call(validity,request)
    pairs={(p.contact_body_1,p.contact_body_2) for p in result.contacts}
    obstacle_pairs={frozenset(p)-{obstacle.id} for p in pairs if obstacle.id in p}
    assert not result.valid and obstacle_pairs=={frozenset({'gemini_camera_link'})},pairs
    print('collision detected only on camera',pairs,flush=True)
finally:
    update(CollisionObject.REMOVE)
assert call(validity,request).valid
print('PASS',flush=True)
node.destroy_node()
rclpy.shutdown()
