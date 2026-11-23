# PiPER-X 工作目录

`piper_x_driver` 负责仿真、MoveIt、控制器和原生 ROS 2 动作接口。`piper_x_tasks` 管理各自独立的测试和正式任务。目前注册项均为 Isaac 测试。

在 `piper_x_driver` 执行 `bash scripts/build.sh` 构建接口。然后执行 `PIPER_DEMO_BARRIER=1 bash scripts/run_isaac.sh`。远程桌面可添加 `--stream --rviz`。看到 `ROS 2 task actions ready` 后，在 `piper_x_tasks` 执行：

```bash
bash run_isaac_task.sh list
bash run_isaac_task.sh run test.empty_motion
bash run_isaac_task.sh run test.obstacle_avoidance
```

接口与仿真边界见 `piper_x_driver/docs/ROS_INTERFACES.md`。服务日志在 driver 的 `.runtime/isaac-services.log`，任务日志在各测试目录的 `logs/`。测试期间由动作服务持有控制权，勿同时从 RViz 直接执行。真机后端尚未接入。
