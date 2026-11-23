"""Reuse official MoveIt configuration with its documented TopicBasedSystem backend."""
import sys
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node
root=Path(get_package_share_directory('agx_arm_moveit'))
sys.path.insert(0,str(root/'launch'))
from _moveit_config_builder import declare_common_args,build_moveit_config

def setup(context):
 config=build_moveit_config(context)
 xml=config.robot_description['robot_description']
 old='<plugin>mock_components/GenericSystem</plugin>'
 assert old in xml
 xml=xml.replace(old,'<plugin>topic_based_ros2_control/TopicBasedSystem</plugin><param name="joint_commands_topic">/piper_x/joint_commands</param><param name="joint_states_topic">/piper_x/physics_joint_states</param>')
 config.robot_description['robot_description']=xml
 joints=[f'joint{i}' for i in range(1,7)]
 controllers={'controller_manager':{'ros__parameters':{'update_rate':200,'arm_controller':{'type':'joint_trajectory_controller/JointTrajectoryController'},'gripper_controller':{'type':'joint_trajectory_controller/JointTrajectoryController'},'joint_state_broadcaster':{'type':'joint_state_broadcaster/JointStateBroadcaster'}}},'arm_controller':{'ros__parameters':{'joints':joints,'command_interfaces':['position'],'state_interfaces':['position','velocity']}},'gripper_controller':{'ros__parameters':{'joints':['gripper'],'command_interfaces':['position'],'state_interfaces':['position','velocity']}}}
 import tempfile,yaml
 f=tempfile.NamedTemporaryFile(mode='w',suffix='.yaml',delete=False);yaml.safe_dump(controllers,f);f.close()
 params=config.to_dict();params.update(use_sim_time=True)
 nodes=[Node(package='robot_state_publisher',executable='robot_state_publisher',parameters=[config.robot_description,{'use_sim_time':True}]),Node(package='moveit_ros_move_group',executable='move_group',parameters=[params]),Node(package='controller_manager',executable='ros2_control_node',parameters=[config.robot_description,f.name,{'use_sim_time':True}])]
 for name in ['joint_state_broadcaster','arm_controller','gripper_controller']:
  nodes.append(Node(package='controller_manager',executable='spawner',arguments=[name,'--controller-manager-timeout','60']))
 from launch.substitutions import LaunchConfiguration
 if LaunchConfiguration('use_rviz').perform(context)=='true':nodes.append(Node(package='rviz2',executable='rviz2',arguments=['-d',str(root/'config/moveit.rviz')],parameters=[params]))
 return nodes
from launch.actions import DeclareLaunchArgument
def generate_launch_description():
 return LaunchDescription(declare_common_args()+[DeclareLaunchArgument('use_rviz',default_value='true'),OpaqueFunction(function=setup)])
