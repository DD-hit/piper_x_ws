import json,time
from pathlib import Path
import rclpy
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
rclpy.init();node=rclpy.create_node('verify_isaac_gripper');state={};results=[]
node.create_subscription(JointState,'/piper_x/isaac_joint_states',lambda msg:state.update(zip(msg.name,msg.position)),10)
client=ActionClient(node,FollowJointTrajectory,'/gripper_controller/follow_joint_trajectory')
try:
 assert client.wait_for_server(timeout_sec=15)
 for opening in [0.04,0.0]:
  goal=FollowJointTrajectory.Goal();goal.trajectory.joint_names=['gripper']
  point=JointTrajectoryPoint();point.positions=[opening];point.time_from_start.sec=2;goal.trajectory.points=[point]
  future=client.send_goal_async(goal);rclpy.spin_until_future_complete(node,future,timeout_sec=15)
  handle=future.result();assert handle and handle.accepted
  future=handle.get_result_async();rclpy.spin_until_future_complete(node,future,timeout_sec=30)
  result=future.result();assert result and result.status==4 and result.result.error_code==0
  end=time.monotonic()+2
  while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.1)
  fingers=[state['gripper_joint1'],state['gripper_joint2']]
  assert abs(fingers[0]-opening/2)<.003 and abs(fingers[1]+opening/2)<.003,fingers
  results.append({'target_opening_m':opening,'actual_finger_positions_m':fingers})
 Path('.runtime/isaac_gripper_verification.json').write_text(json.dumps(results,indent=2))
 print(json.dumps(results,indent=2),flush=True)
finally:node.destroy_node();rclpy.shutdown()
