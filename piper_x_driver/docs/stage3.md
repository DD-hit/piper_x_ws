# 第三阶段：独立启动仿真和机械臂服务

在服务器的两个终端中分别执行。先启动 Isaac：

```bash
cd /home/rl/workspace/piper_x_ws/piper_x_driver
bash scripts/run_isaac_scene.sh --headless
```

看到 `PIPER_ISAAC_READY` 后启动服务：

```bash
cd /home/rl/workspace/piper_x_ws/piper_x_driver
bash scripts/run_isaac_moveit.sh use_rviz:=false
```

在远程桌面中可使用 `use_rviz:=true`。业务程序单独启动，先加载 `scripts/env.sh` 并设置 `ROS_DOMAIN_ID=74`，再等待所需的规划服务、动作服务和关节反馈。可用 `scripts/verify_mock.py --backend isaac` 和 `scripts/verify_isaac_gripper.py` 验证接口。这两个脚本会驱动仿真运动。

也可以用 `bash scripts/run_isaac.sh --stream --rviz` 一起启动。此时 Isaac 和服务的输出分别在 `.runtime/isaac-scene.log` 和 `.runtime/isaac-services.log`。独立启动时，在各自的终端查看输出。

机械臂服务启动前最多等待 15 秒的有效物理反馈。运行期间，若 5 秒未收到时间戳更新且关节齐全的反馈，服务退出并关闭本次启动的组件。分别启动时，Isaac 和服务均限制为单实例。业务程序应等待动作结果并检查实际状态。

故障后先查对应日志，确认 Isaac 已恢复步进，再启动机械臂服务。读取最新状态并重新规划后，再发下一条任务命令。服务不会自动重发运动。正常停止时，先结束业务程序，再用 Ctrl+C 停止机械臂服务，最后停止 Isaac。
