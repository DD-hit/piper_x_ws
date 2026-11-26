# Isaac Sim

从工作区根目录加载 `scripts/env.sh`，运行
`ros2 launch piper_x_bringup simulation.launch.py`。
此入口启动带 Gemini 腕部相机的抓取场景，并自动加载 MoveIt 碰撞物体。

交互式串流可通过 `ros2 launch piper_x_bringup simulation.launch.py stream:=true` 启动。
服务使用 ROS 域 74，任务入口为 `piper_x_tasks/run_isaac_task.sh`。

## Isaac 串流依赖

从工作区根目录运行 `bash scripts/install_isaac_deps.sh`。脚本使用 Isaac
自带的 Python，按 `scripts/isaac-requirements.txt` 安装到 `.deps/isaac/`。
ROS 2 任务仍使用工作区的 `.venv/`，运行日志和仿真状态写入 `.runtime/`。
