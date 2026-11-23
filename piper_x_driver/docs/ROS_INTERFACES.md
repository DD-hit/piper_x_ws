# PiPER-X 仿真任务与运动接口

在 `piper_x_driver` 运行 `bash scripts/run_isaac.sh`。挡板测试加环境变量 `PIPER_DEMO_BARRIER=1`；远程观察可加 `--rviz --stream`。等待 `ROS 2 task actions ready` 后，在 `piper_x_tasks` 执行 `bash run_isaac_task.sh run test.empty_motion`。服务器上现有的 Isaac 测试运行在 ROS 域 74。

`/piper_x/run_task` 接收 `piper_x_interfaces/action/RunTask`。Goal 只填写 `name`。driver 从注册表解析入口、独占整次任务并启动子进程。Result 包含运行编号 `run_id`、任务日志路径 `log_path` 和结果码。客户端 Ctrl+C 请求取消。提交方异常断开时，已提交任务继续由 driver 管理；可由持有动作目标的客户端取消。任务子进程退出或发生异常时，driver 结束其执行通道。取消后的下游动作终态无法确认时进入故障锁定。进程启动后没有任务租约或运行时间上限。

注册任务使用 `native_client.Arm` 调用 driver。通道只传给本次启动的任务，无需任务代码填写控制凭证。任务运行时公开运动动作返回 `BUSY`。空闲时终端可以直接调用，例如：

```bash
source scripts/env.sh
export ROS_DOMAIN_ID=74
ros2 action send_goal /piper_x/move_t piper_x_interfaces/action/MoveT '{translation: {header: {frame_id: base_link}, vector: {z: 0.01}}, velocity_scale: 0.1}' --feedback
ros2 topic echo /piper_x/status --once --qos-durability transient_local
```

`status.owner` 为正在执行的任务名，`active_action` 为当前动作。手动动作只占用动作执行期间。`set_enabled` 控制 Isaac 运动许可。接口包变更后需要重新加载 driver 的构建环境。

`move_j` 为直接关节目标，`move_p` 的 `AVOID_OBSTACLES` 使用 MoveIt；`DIRECT` 对 TCP 目标求逆解后执行关节插值，不做场景绕障。绝对位姿使用 `base_link`，直线与圆弧保持朝向。`move_t` 支持基座或工具坐标系下 XYZ 平移，`set_gripper` 以米表示开度。MoveJ/P/L/C/T 均支持独立的速度与加速度比例，详见 [配置说明](CONFIGURATION.md)。RViz、直接访问 MoveIt 或底层控制器的客户端仍可绕过本任务入口。真机后端尚未接入。
