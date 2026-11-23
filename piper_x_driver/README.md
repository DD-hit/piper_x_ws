# PiPER-X driver

本项目提供原生 ROS 2 接口，内部复用官方模型、MoveIt 和标准轨迹控制器。目前执行后端为 Isaac 物理仿真；真机 SDK/CAN 后端待接入和验收。

## 环境约定

在本目录执行 `bash scripts/build.sh`。`scripts/env.sh` 加载 `/opt/ros/humble`、可选的 `.venv` 和本工作空间 `install`，避免混用 Conda 或其他 ROS 工作空间。

当前部署包含机器相关配置：

- Isaac 安装路径在 `scripts/run_isaac_scene.sh` 中为 `/home/rl/isaacsim`。
- GPU 在 `sim/isaac/run_scene.py` 中选择 GPU 1；需要对应硬件或调整配置。
- 串流公布地址在同一场景脚本中配置；迁移服务器需修改。TCP 49100 和 UDP 47998 必须可达。
- 仿真脚本使用 ROS 域 74、本机 ROS 通信；mock 演示默认域 73。
- 项目 Python 环境需提供 colcon、PyYAML、NumPy、SciPy 和 ROS Python 依赖。真机 SDK 尚未作为执行后端部署。
- Isaac 串流补充依赖位于未提交的 `.runtime/isaac-python-deps`，恢复方法见 [第二阶段记录](docs/stage2.md)。

仓库不是完整系统安装器。官方依赖通过子模块获取，Isaac 使用其自带 Python，不安装到项目虚拟环境。

## 启动

```bash
# 无挡板，用于空载、直线、圆弧及故障请求测试
PIPER_DEMO_BARRIER=0 bash scripts/run_isaac.sh --stream --rviz
```

等待 `ROS 2 task actions ready`。`--rviz` 需在图形远程桌面运行，`--stream` 提供 WebRTC 画面；两者可分别省略。挡板测试需退出后用 `PIPER_DEMO_BARRIER=1` 重新启动。运行任务见 [tasks](../piper_x_tasks/README.md)。

## 对外接口

| 接口 | 类型 | 语义 |
| --- | --- | --- |
| `/piper_x/run_task` | `RunTask` Action | 按注册名执行完整任务，结果带运行编号和日志路径 |
| `/piper_x/move_j` | `MoveJ` Action | 六轴关节目标，直接插值，不绕障 |
| `/piper_x/move_p` | `MoveP` Action | 绝对 TCP 位姿，显式选择 DIRECT 或 AVOID_OBSTACLES |
| `/piper_x/move_l` | `MoveL` Action | 固定朝向的绝对直线运动 |
| `/piper_x/move_c` | `MoveC` Action | 固定朝向的三点圆弧，需已在起点 |
| `/piper_x/move_t` | `MoveT` Action | 基座或工具坐标系下 XYZ 相对直线平移 |
| `/piper_x/set_gripper` | `SetGripper` Action | 总开度，单位 m |
| `/piper_x/set_enabled` | `std_srvs/SetBool` Service | 仿真运动许可 |
| `/piper_x/status` | `ArmStatus` Topic | 就绪、忙碌、故障、使能、任务名及当前动作 |

自定义类型均在 `piper_x_interfaces` 中。绝对位姿当前要求 `base_link` 和单位四元数。DIRECT 采用 IK 加关节插值，不复刻官方固件算法；避障模式使用 OMPL/RRTConnect。直线、圆弧和增量在仿真中检查整段路径，失败时不执行部分路径。

## 终端单动作与状态查询

空闲时直接调用，无 token、owner 请求字段或控制权租约：

```bash
source scripts/env.sh
export ROS_DOMAIN_ID=74
ros2 action send_goal /piper_x/move_t piper_x_interfaces/action/MoveT \
  '{translation: {header: {frame_id: base_link}, vector: {z: 0.01}}, velocity_scale: 0.2, acceleration_scale: 0.1}' --feedback
ros2 topic echo /piper_x/status --once --qos-durability transient_local
ros2 run tf2_ros tf2_echo base_link tcp_link
```

比例参数相对关节模型限值；0 表示采用默认值。配置位于 `config/driver.yaml`，启动时加载。详见 [配置说明](docs/CONFIGURATION.md)。

状态 `0/1/2/3` 分别为 STARTING/READY/BUSY/FAULT。完整任务执行期间 `owner` 为任务名，手动动作时为空；判断忙碌应看 state，不能只看 owner。READY 不代替 enabled 检查。

## 互斥、停止与验收边界

driver 从任务开始到退出全程持锁；其他公开动作返回 BUSY。取消或异常退出时先结束下游动作，停止不明则保持 FAULT。提交端异常断开不会自动终止已提交任务。直接调用 MoveIt、底层控制器或 RViz 执行仍可绕过此锁。

无活动会话时运行 `source scripts/env.sh`、`python3 scripts/acceptance.py` 自动验收。日志和报告见根目录 [README](../README.md#自动验收与日志)。mock 启动与 `docs/stage*.md` 保留为历史调试资料，不代表当前真机能力。

已验证的是仿真空载运动、指定路径、状态和专项故障处理。真实固件、CAN 通信、停止距离、负载和实际抓取仍需单独验证。
