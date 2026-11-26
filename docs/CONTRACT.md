# 仿真接口契约与真机边界

所有动作由 `/piper_x` 提供；`run_task` 持有整次注册任务锁，公开运动请求在此期间返回 BUSY。注册任务代码、参数、验收条件和日志位于独立的 tasks 项目。driver 使用单个 Motion 后端执行动作。

## 运动语义

MoveJ 接收六轴 rad 目标，关节插值，不做场景绕障。MoveP.DIRECT 对绝对 TCP 位姿求 IK，再关节插值，不做场景绕障；它不复刻真机固件的运动算法。MoveP.AVOID_OBSTACLES 使用 OMPL/RRTConnect 规划并做场景碰撞检查。两者不互相降级。当前绝对位姿均要求 base_link，朝向使用单位四元数。

MoveL 固定朝向沿直线；MoveC 固定朝向沿三点圆弧，当前位置必须在起点附近；MoveT 为基座或工具坐标系下 XYZ 合成直线增量。笛卡尔路径在仿真中检查碰撞，不允许部分路径执行。SetGripper 使用总开度 m。

velocity_scale 范围 (0,1]。MoveP 避障模式使用 MoveIt 的速度和加速度缩放；笛卡尔模式对已有轨迹进行减速，速度按 scale、加速度按 scale²；关节直插以官方 URDF 关节速度上限乘 scale 为峰值参考，采用零终点速度插值和至少 1 秒时长。该参数不是 m/s，也不是精确的整段持续时间。位置反馈误差与实际峰值速度仍需真实硬件验收。

## 官方资源复用

官方模型、SRDF、MoveIt 配置和 JointTrajectoryController 已复用。当前完成 Isaac 后端。真机 MoveIt 路线将复用轨迹控制器的参考输出，由唯一的 pyAgxArm 实例写 CAN，并把 SDK 真实状态读回控制器。官方直接 move_j/p/l/c、enable/disable、夹爪宽度与诊断接口存在，但当前未部署真机适配；MIT、JS、CPV、配置写入和电子急停/复位同样尚未接入或验收，不能假定仿真已经覆盖固件。

设备后端边界是：连接、状态读取、周期关节写入、取消和停止确认、使能。任务入口与规划器不得另建 CAN 写入通道。停止确认失败必须保持 FAULT；恢复不重发旧目标。当前 RViz/底层控制器仍能绕过公开入口互斥。

## 验收

空闲时在 driver 执行 `source scripts/env.sh` 后运行 `python3 scripts/acceptance.py`。自动启动无挡板与挡板两类场景并生成 `.runtime/acceptance-时间/report.json`。每次保存独立 driver 和任务日志。该入口覆盖注册的正常路径与故障请求；进程强杀、反馈断流、恢复和性能基准的历史结果见 tasks 的 `tests/.runner_validation/result.log` 与 driver 的 `.runtime/run-task-loss-*`，未伪装成每次都会重复的自动验收。真机验收单独执行。
