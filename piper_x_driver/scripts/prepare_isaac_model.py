"""Expand the same official xacro as MoveIt, resolving meshes for Isaac's importer.
The generated URDF is an import artifact; upstream xacro remains the source.
"""
from pathlib import Path
import xml.etree.ElementTree as ET
import xacro
from ament_index_python.packages import get_package_share_directory
root=Path(__file__).resolve().parents[1]
share=Path(get_package_share_directory('agx_arm_moveit'))
doc=xacro.process_file(str(share/'config/agx_arm.urdf.xacro'),mappings={'arm_type':'piper_x','effector_type':'agx_gripper','revo2_type':'left','tcp_offset_xyz':'0 0 0','tcp_offset_rpy':'0 0 0'})
robot=ET.fromstring(doc.toxml())
for e in list(robot):
 if e.tag=='ros2_control' or (e.tag=='joint' and e.get('name')=='gripper') or (e.tag=='link' and e.get('name')=='gripper_link'):robot.remove(e)
for j in robot.findall('joint'):
 for mimic in j.findall('mimic'):j.remove(mimic)
for mesh in robot.findall('.//mesh'):
 uri=mesh.get('filename')
 if uri.startswith('package://'):
  package,relative=uri[10:].split('/',1)
  path=Path(get_package_share_directory(package))/relative
  if not path.is_file():raise FileNotFoundError(path)
  mesh.set('filename',str(path))
ledger='<!-- Official PiPER-X xacro expansion; SI metres/radians, source axes/origins/limits/inertials unchanged. ROS aggregate gripper is a nonphysical command coordinate, removed with its empty link and mimic tags for independent physics finger control. Map opening q to gripper_joint1=q/2, gripper_joint2=-q/2, exactly as upstream mimic multipliers. No dynamics calibration claimed. -->\n'
output=root/'.runtime/current_model/piper_x.urdf';output.parent.mkdir(parents=True,exist_ok=True)
output.write_text(ledger+ET.tostring(robot,encoding='unicode'))
print('Prepared official PiPER-X import:',output)
