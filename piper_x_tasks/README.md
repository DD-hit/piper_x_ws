# PiPER-X 任务注册表

执行 `bash run_isaac_task.sh list` 查看注册任务，执行 `bash run_isaac_task.sh run 任务名` 提交给 driver。每次运行在独立进程内执行，日志在入口目录的 `logs/<run_id>.log`。终端 Ctrl+C 会请求取消。无挡板场景适合直线、圆弧和空载测试；挡板测试需加载 `demo_barrier`。

新增任务时，在 `tests/` 或 `production/` 下放置入口，在 `registry.yaml` 登记唯一名称、`kind`、`environments`、`entry`、`description`。通过 `with Arm() as arm` 使用 driver 分配的本次任务通道。运动互斥由 driver 负责。历史脚本放在 `archives/`，正式任务数量为零。完整接口说明见同级 driver 的 `docs/ROS_INTERFACES.md`。
