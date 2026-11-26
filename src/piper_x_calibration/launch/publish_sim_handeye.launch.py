from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    config=PathJoinSubstitution([FindPackageShare('piper_x_calibration'),'config','simulation','gemini_handeye.yaml'])
    return LaunchDescription([Node(package='piper_x_calibration', executable='publish_handeye.py', arguments=['--config',config], output='screen')])
