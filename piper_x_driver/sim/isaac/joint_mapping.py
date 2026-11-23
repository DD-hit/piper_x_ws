"""Map official ROS aggregate opening to physical fingers, using upstream mimic signs."""
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
class Mapping(Node):
 def __init__(self):
  super().__init__('piper_x_isaac_joint_mapping')
  self.command=self.create_publisher(JointState,'/piper_x/isaac_joint_commands',10)
  self.state=self.create_publisher(JointState,'/piper_x/physics_joint_states',10)
  self.create_subscription(JointState,'/piper_x/joint_commands',self.on_command,10)
  self.create_subscription(JointState,'/piper_x/isaac_joint_states',self.on_state,10)
 def on_command(self,msg):
  out=JointState();out.header=msg.header
  for name,q in zip(msg.name,msg.position):
   if name=='gripper':
    out.name.extend(['gripper_joint1','gripper_joint2']);out.position.extend([q/2,-q/2])
   elif name in [f'joint{i}' for i in range(1,7)]:
    out.name.append(name);out.position.append(q)
  self.command.publish(out)
 def on_state(self,msg):
  data=dict(zip(msg.name,msg.position));vel=dict(zip(msg.name,msg.velocity))
  required=[f'joint{i}' for i in range(1,7)]+['gripper_joint1','gripper_joint2']
  if not all(name in data and math.isfinite(data[name]) for name in required):return
  out=JointState();out.header=msg.header
  out.name=[f'joint{i}' for i in range(1,7)]+['gripper']
  out.position=[data[n] for n in out.name[:6]]+[data['gripper_joint1']-data['gripper_joint2']]
  out.velocity=[vel.get(n,0.) for n in out.name[:6]]+[vel.get('gripper_joint1',0.)-vel.get('gripper_joint2',0.)]
  self.state.publish(out)
rclpy.init();node=Mapping()
try:rclpy.spin(node)
except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):pass
finally:node.destroy_node();rclpy.try_shutdown()
