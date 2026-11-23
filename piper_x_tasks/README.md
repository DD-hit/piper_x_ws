# PiPER-X tasks

这里存放相互独立的测试与业务入口。driver 接收注册名，启动独立进程并管理整次任务的运动互斥。目前正式业务任务为零。

## 运行

先按 [总说明](../README.md) 启动相应场景，等待 driver 就绪，再在本目录执行：

```bash
bash run_isaac_task.sh list
bash run_isaac_task.sh run test.empty_motion
```

启动器通过 ROS 2 `/piper_x/run_task` 提交任务，不在本地直接运行入口脚本。使用包装脚本加载新构建的接口环境，避免接口版本不一致。

| 注册名 | 场景 | 观察内容 |
| --- | --- | --- |
| `test.empty_motion` | 无挡板 | 关节目标、DIRECT 位姿目标、XYZ 增量和夹爪开合 |
| `test.cartesian_line` | 无挡板 | 绝对直线、基座及工具 XYZ 增量的路径采样误差 |
| `test.cartesian_arc` | 无挡板 | 三点圆弧和路径、朝向误差 |
| `test.fault_handling` | 无挡板 | 手动请求 BUSY、失能拒绝、非法/不可达目标及恢复 |
| `test.obstacle_avoidance` | 有挡板 | 检查碰撞前提，再以两个绝对 TCP 位姿请求绕障 |

故障测试中的预期错误不等于测试失败，以最终 PASS/FAIL 为准。该注册测试不自动暂停 Isaac；断流、强杀、任务取消和延迟测量是单独专项验收。挡板规划为随机搜索，失败应保留日志。

## 结果、取消与日志

终端显示 `run_id`、结果和日志绝对路径。每次日志在对应任务的 `logs/<run_id>.log`；路径测试另写 `logs/path-verification.json`，此文件会被下一次运行覆盖。完整 driver 日志位于 driver 的 `.runtime/isaac-services.log`。

按 `Ctrl+C` 请求取消，等待终态后再运行下一任务。任务之间的等待不受租约限制。若终端或网络异常断开，driver 仍负责已提交任务的执行。勿同时从 RViz 或底层控制器发送运动。

## 新增任务

在 `tests/` 或 `production/` 创建独立目录和 `main.py`，在 `registry.yaml` 添加唯一注册名、`kind`、`environments`、`entry` 和 `description`。当前执行环境仅 isaac。入口位置必须属于相应类别目录。

任务示例：

```python
from native_client import Arm
from piper_x_interfaces.action import MoveT
from geometry_msgs.msg import Vector3Stamped

with Arm() as arm:
    delta = Vector3Stamped()
    delta.header.frame_id = 'base_link'
    delta.vector.z = 0.01
    arm.run(MoveT, 'move_t', translation=delta,
            velocity_scale=0.2, acceleration_scale=0.1)
```

driver 为子进程分配内部执行通道，任务不填写控制凭证，也不负责设备重连、停止确认或下游反馈超时。任务负责步骤、目标和业务验收，失败默认退出。`native_client` 由启动环境提供，直接执行入口文件不受支持。历史脚本在 `archives/`，不作为当前调用示例。
