import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def start_simulation(context):
    root = Path(os.environ["PIPER_X_WS_ROOT"])
    command = ["bash", str(root / "scripts/run_isaac.sh")]
    if LaunchConfiguration("stream").perform(context) == "true":
        command.append("--stream")
    command.append("wrist_attachment:=" + LaunchConfiguration("wrist_attachment").perform(context))
    return [
        ExecuteProcess(
            cmd=command,
            additional_env={
                "PIPER_GRASP_SCENE": LaunchConfiguration("grasp_scene"),
                "PIPER_DEMO_BARRIER": "0",
                "PIPER_EXECUTION_ENV": "isaac",
            },
            sigterm_timeout="25",
            output="screen",
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("grasp_scene", default_value="1"),
        DeclareLaunchArgument("wrist_attachment", default_value="gemini336l"),
        DeclareLaunchArgument("stream", default_value="false", choices=["true", "false"]),
        OpaqueFunction(function=start_simulation),
    ])
