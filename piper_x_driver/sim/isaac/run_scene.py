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
if os.environ.get('PIPER_DEMO_BARRIER') == '1':
 import json
 from isaacsim.core.api.objects import FixedCuboid
 barrier=json.loads(Path(__file__).with_name('barrier.json').read_text())
 world.scene.add(FixedCuboid(prim_path='/World/demo_barrier', position=np.array(barrier['center']),
                             size=1.0, scale=np.array(barrier['size']), color=np.array([0.8,0.3,0.1])))
light=UsdLux.DistantLight.Define(stage,'/World/Light');light.CreateIntensityAttr(2000)
world.reset()
print('PIPER_DOF_NAMES',robot.dof_names,flush=True)
robot.apply_action(ArticulationAction(joint_positions=np.zeros(len(robot.dof_names))))
K=og.Controller.Keys
og.Controller.edit({'graph_path':'/World/PiperROS','evaluator_name':'execution'},{
 K.CREATE_NODES:[('Tick','omni.graph.action.OnPlaybackTick'),('Time','isaacsim.core.nodes.IsaacReadSimulationTime'),('Clock','isaacsim.ros2.bridge.ROS2PublishClock'),('State','isaacsim.ros2.bridge.ROS2PublishJointState'),('Cmd','isaacsim.ros2.bridge.ROS2SubscribeJointState'),('Drive','isaacsim.core.nodes.IsaacArticulationController')],
 K.CONNECT:[('Tick.outputs:tick','Clock.inputs:execIn'),('Time.outputs:simulationTime','Clock.inputs:timeStamp'),('Tick.outputs:tick','State.inputs:execIn'),('Time.outputs:simulationTime','State.inputs:timeStamp'),('Tick.outputs:tick','Cmd.inputs:execIn'),('Cmd.outputs:execOut','Drive.inputs:execIn'),('Cmd.outputs:jointNames','Drive.inputs:jointNames'),('Cmd.outputs:positionCommand','Drive.inputs:positionCommand')],
 K.SET_VALUES:[('Clock.inputs:topicName','/clock'),('State.inputs:topicName','/piper_x/isaac_joint_states'),('State.inputs:targetPrim',[articulation_path]),('Cmd.inputs:topicName','/piper_x/isaac_joint_commands'),('Drive.inputs:targetPrim',[articulation_path])]
})
for _ in range(30):world.step(render=True)
from isaacsim.core.utils.viewports import set_camera_view
set_camera_view(eye=np.array([1.2,1.2,0.85]),target=np.array([0.,0.,0.35]))
print('PIPER_ISAAC_READY',flush=True)
try:
 while app.is_running():
  world.step(render=True)
  request=Path('.runtime/capture.request')
  if request.exists():
   request.unlink()
   from omni.kit.viewport.utility import get_active_viewport, capture_viewport_to_file
   capture_viewport_to_file(get_active_viewport(),str(Path('.runtime/isaac_scene.png').resolve()))
finally:
 app.close()
