"""Import the current official PiPER-X URDF and expose physics through ROS 2."""
import argparse
from pathlib import Path
parser=argparse.ArgumentParser()
parser.add_argument('--headless',action='store_true')
parser.add_argument('--stream',action='store_true')
args=parser.parse_args()
import sys
sys.path.insert(0,str(Path(".runtime/isaac-python-deps").resolve()))
from isaacsim import SimulationApp
app=SimulationApp({'headless':args.headless,'active_gpu':1,'physics_gpu':1,'multi_gpu':False,'width':1280,'height':720,'hide_ui':False})
from isaacsim.core.utils.extensions import enable_extension
if args.stream:
 app.set_setting('/app/livestream/publicEndpointAddress','100.73.76.90')
 app.set_setting('/app/livestream/port',49100)
 app.set_setting('/app/livestream/fixedHostPort',47998)
 app.set_setting('/app/livestream/nvcf/quitOnSessionEnded',False)
 enable_extension('omni.services.livestream.nvcf')
enable_extension('isaacsim.ros2.bridge')
app.update()
from isaacsim.core.api import World
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.types import ArticulationAction
from pxr import UsdPhysics
import omni.graph.core as og
import numpy as np
enable_extension('isaacsim.asset.importer.urdf')
import omni.kit.commands
import omni.usd
from pxr import UsdLux
omni.usd.get_context().new_stage()
status,cfg=omni.kit.commands.execute('URDFCreateImportConfig')
cfg.fix_base=True
cfg.merge_fixed_joints=True
cfg.import_inertia_tensor=True
cfg.make_default_prim=True
cfg.create_physics_scene=True
status,robot_path=omni.kit.commands.execute('URDFParseAndImportFile',urdf_path=str(Path('.runtime/current_model/piper_x.urdf').resolve()),import_config=cfg)
assert status, 'Official URDF import failed'
world=World(stage_units_in_meters=1.0,physics_dt=1/120,rendering_dt=1/60)
stage=world.stage
# Simulation-only position drives; retain the official 10 N finger effort limit.
for prim in stage.Traverse():
 if prim.GetName() in ('gripper_joint1','gripper_joint2') and prim.IsA(UsdPhysics.PrismaticJoint):
  drive=UsdPhysics.DriveAPI.Apply(prim,'linear')
  drive.CreateStiffnessAttr(2000.0)
  drive.CreateDampingAttr(20.0)
  drive.CreateMaxForceAttr(10.0)
root_paths=[str(p.GetPath()) for p in stage.Traverse() if p.HasAPI(UsdPhysics.ArticulationRootAPI)]
assert len(root_paths)==1,root_paths
articulation_path=root_paths[0]
print('PIPER_ARTICULATION',articulation_path,flush=True)
robot=world.scene.add(SingleArticulation(prim_path=robot_path,name='piper_x'))
world.scene.add_default_ground_plane(z_position=-0.02)
import os
grasp_cube=None
if os.environ.get('PIPER_GRASP_SCENE') == '1':
 from isaacsim.core.api.objects import DynamicCuboid, FixedCuboid
 from isaacsim.core.api.materials import PhysicsMaterial
 material=PhysicsMaterial(prim_path='/World/GraspMaterial', static_friction=1.0, dynamic_friction=0.8, restitution=0.0)
 cube_material=PhysicsMaterial(prim_path='/World/GraspCubeMaterial', static_friction=1.3, dynamic_friction=1.1, restitution=0.0)
 world.scene.add(FixedCuboid(name='grasp_table',prim_path='/World/GraspTable',position=np.array([0.40,0.,0.04]),
                            size=1.,scale=np.array([0.4,0.5,0.08]),color=np.array([0.3,0.35,0.4]),physics_material=material))
 grasp_cube=world.scene.add(DynamicCuboid(name='grasp_cube',prim_path='/World/GraspCube',position=np.array([0.35,-0.08,0.095]),
                             size=0.03,mass=0.03,color=np.array([0.9,0.3,0.1]),physics_material=cube_material))
 world.scene.add(FixedCuboid(name='place_marker',prim_path='/World/PlaceMarker',position=np.array([0.35,0.08,0.0805]),
                            size=1.,scale=np.array([0.07,0.07,0.001]),color=np.array([0.2,0.7,0.3])))
 from isaacsim.core.prims import RigidPrim
 finger_paths = {prim.GetName(): str(prim.GetPath()) for prim in stage.Traverse()
                 if prim.GetName() in ('gripper_link1', 'gripper_link2') and prim.HasAPI(UsdPhysics.RigidBodyAPI)}
 finger_views = [world.scene.add(RigidPrim(finger_paths[name], name=name + '_contact',
                 reset_xform_properties=False, track_contact_forces=True,
                 contact_filter_prim_paths_expr=['/World/GraspCube'], max_contact_count=32))
                 for name in ('gripper_link1', 'gripper_link2')]
if os.environ.get('PIPER_DEMO_BARRIER') == '1':
 import json
 from isaacsim.core.api.objects import FixedCuboid
 barrier=json.loads(Path(__file__).with_name('barrier.json').read_text())
 world.scene.add(FixedCuboid(prim_path='/World/demo_barrier', position=np.array(barrier['center']),
                             size=1.0, scale=np.array(barrier['size']), color=np.array([0.8,0.3,0.1])))
light=UsdLux.DistantLight.Define(stage,'/World/Light');light.CreateIntensityAttr(2000)
wrist_attachment = os.environ.get('PIPER_WRIST_ATTACHMENT') or (
 'gemini336l' if os.environ.get('PIPER_GEMINI') == '1' else 'none')
if wrist_attachment not in ('none','gemini336l'):
 raise ValueError(f'Unknown wrist attachment: {wrist_attachment}')
gemini_enabled = wrist_attachment == 'gemini336l'
if gemini_enabled or os.environ.get('PIPER_GEMINI_VISUAL') == '1':
 from gemini_visual import add_gemini_visual
 gemini_attachment = add_gemini_visual(stage)
if gemini_enabled:
 from gemini_collision import add_gemini_collisions
 gemini_collision_paths = add_gemini_collisions(stage, gemini_attachment)
world.reset()
print('PIPER_DOF_NAMES',robot.dof_names,flush=True)
robot.apply_action(ArticulationAction(joint_positions=np.zeros(len(robot.dof_names))))
K=og.Controller.Keys
og.Controller.edit({'graph_path':'/World/PiperROS','evaluator_name':'execution'},{
 K.CREATE_NODES:[('Tick','omni.graph.action.OnPlaybackTick'),('Time','isaacsim.core.nodes.IsaacReadSimulationTime'),('Clock','isaacsim.ros2.bridge.ROS2PublishClock'),('State','isaacsim.ros2.bridge.ROS2PublishJointState'),('Cmd','isaacsim.ros2.bridge.ROS2SubscribeJointState'),('Drive','isaacsim.core.nodes.IsaacArticulationController')],
 K.CONNECT:[('Tick.outputs:tick','Clock.inputs:execIn'),('Time.outputs:simulationTime','Clock.inputs:timeStamp'),('Tick.outputs:tick','State.inputs:execIn'),('Time.outputs:simulationTime','State.inputs:timeStamp'),('Tick.outputs:tick','Cmd.inputs:execIn'),('Cmd.outputs:execOut','Drive.inputs:execIn'),('Cmd.outputs:jointNames','Drive.inputs:jointNames'),('Cmd.outputs:positionCommand','Drive.inputs:positionCommand')],
 K.SET_VALUES:[('Clock.inputs:topicName','/clock'),('State.inputs:topicName','/piper_x/isaac_joint_states'),('State.inputs:targetPrim',[articulation_path]),('Cmd.inputs:topicName','/piper_x/isaac_joint_commands'),('Drive.inputs:targetPrim',[articulation_path])]
})
if gemini_enabled:
 from gemini_sensor import add_gemini_sensors
 gemini_products = add_gemini_sensors(stage, gemini_attachment)
 from gemini_fixture_visual import GeminiFixtureVisual
 gemini_fixture_visual = GeminiFixtureVisual(stage)
elif os.environ.get('PIPER_WRIST_CAMERA') == '1':
 from wrist_camera import add_wrist_camera
 wrist_render_product = add_wrist_camera(stage)
if os.environ.get('PIPER_HANDEYE_BOARD'):
 from handeye_board import add_handeye_board
 add_handeye_board(stage, os.environ['PIPER_HANDEYE_BOARD'])
for _ in range(30):world.step(render=True)
from isaacsim.core.utils.viewports import set_camera_view
set_camera_view(eye=np.array([1.2,1.2,0.85]),target=np.array([0.,0.,0.35]))
print('PIPER_ISAAC_READY',flush=True)
if grasp_cube:
 import json, time
 from pxr import Gf
 last_grasp_telemetry=0.
 finger_indices = [robot.dof_names.index(name) for name in ('gripper_joint1', 'gripper_joint2')]
 Path('.runtime/grasp-telemetry.jsonl').write_text('')
try:
 while app.is_running():
  if gemini_enabled:
   gemini_fixture_visual.update()
  world.step(render=True)
  if grasp_cube and time.monotonic()-last_grasp_telemetry>0.05:
   last_grasp_telemetry=time.monotonic()
   position,orientation=grasp_cube.get_world_pose()
   telemetry=Path('.runtime/grasp-state.tmp')
   poses = [view.get_world_poses() for view in finger_views]
   centers = [pose[0][0] for pose in poses]
   rotation = Gf.Quatd(float(poses[0][1][0][0]), Gf.Vec3d(*map(float, poses[0][1][0][1:])))
   local_offset = rotation.GetInverse().Transform(Gf.Vec3d(*map(float, position-np.mean(centers,axis=0))))
   actual = robot.get_joint_positions()[finger_indices]
   targets = robot._articulation_view._physics_view.get_dof_position_targets()[0]
   row = dict(wall_time=time.time(), sim_time=world.current_time, position=position.tolist(),
              orientation=orientation.tolist(), opening_m=float(actual[0]-actual[1]),
              target_fingers_m=[float(targets[index]) for index in finger_indices],
              contact_n=[float(np.linalg.norm(view.get_contact_force_matrix(dt=1/120)[0, 0])) for view in finger_views],
              finger_origins_m=[center.tolist() for center in centers],
              cube_offset_gripper_m=list(local_offset),
              cube_offset_world_m=(position-np.mean(centers,axis=0)).tolist())
   telemetry.write_text(json.dumps(row))
   telemetry.replace('.runtime/grasp-state.json')
   with Path('.runtime/grasp-telemetry.jsonl').open('a') as trace:
    trace.write(json.dumps(row)+'\n')
  request=Path('.runtime/capture.request')
  if request.exists():
   request.unlink()
   from omni.kit.viewport.utility import get_active_viewport, capture_viewport_to_file
   capture_viewport_to_file(get_active_viewport(),str(Path('.runtime/isaac_scene.png').resolve()))
finally:
 app.close()
