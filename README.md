# PiPER-X 仿真工作区

基于官方模型、ROS 2 Humble、MoveIt 2 和 Isaac Sim 的 PiPER-X 控制与测试项目。已实现原生 ROS 2 运动接口、整次任务互斥、配置管理及仿真验收。真机 SDK/CAN 后端尚未接入，当前测试结果不代表真机验收通过。

| 项目 | 职责 |
| --- | --- |
| [piper_x_driver](piper_x_driver/README.md) | ROS 2 接口、运动执行、MoveIt、Isaac、状态和故障处理 |
| [piper_x_tasks](piper_x_tasks/README.md) | 独立测试任务、注册表和任务提交入口；当前无正式业务任务 |

外层是总工作目录。请在 `piper_x_driver` 内构建，不在外层统一执行 `colcon build`。

## 环境与获取

已验证环境为 Ubuntu 22.04、ROS 2 Humble、Isaac Sim 5.0.0-rc.45 和 NVIDIA GPU。Isaac、ROS 2、项目 Python 依赖和串流客户端需单独安装，克隆仓库不会安装它们。

```bash
git clone --recurse-submodules https://github.com/DD-hit/piper_x_ws.git
cd piper_x_ws/piper_x_driver
bash scripts/build.sh
```

已克隆但缺官方依赖时，在仓库根目录执行 `git submodule update --init --recursive`。子模块固定官方提交，不自动更新。当前启动脚本针对服务器环境配置，迁移机器前请检查 [driver 的环境说明](piper_x_driver/README.md#环境约定)。

## 快速使用

以下命令按现有服务器路径编写。在远程桌面终端启动无挡板场景：

```bash
cd /home/rl/workspace/piper_x_ws/piper_x_driver
PIPER_DEMO_BARRIER=0 bash scripts/run_isaac.sh --stream --rviz
```

等待 `ROS 2 task actions ready`。Mac 的 Isaac Sim WebRTC Streaming Client 连接服务器 Tailscale 地址，即可查看物理仿真。`--stream` 和 `--rviz` 都可省略，省略后仍可执行任务。

另一个服务器终端执行：

```bash
cd /home/rl/workspace/piper_x_ws/piper_x_tasks
bash run_isaac_task.sh list
bash run_isaac_task.sh run test.empty_motion
bash run_isaac_task.sh run test.cartesian_line
bash run_isaac_task.sh run test.cartesian_arc
bash run_isaac_task.sh run test.fault_handling
```

每项结束后再执行下一项。任务运行期间勿从 RViz 或底层控制器直接发送运动。

## 挡板避障场景

先在 driver 终端按 `Ctrl+C`，等待退出完成，再启动挡板场景：

```bash
PIPER_DEMO_BARRIER=1 bash scripts/run_isaac.sh --stream --rviz
```

另一个终端运行 `bash run_isaac_task.sh run test.obstacle_avoidance`。该任务先验证挡板碰撞前提，再向 MoveP 提交两侧绝对 TCP 位姿，由 MoveIt 搜索绕行路径。规划失败时停止，不自动降级为直达。随机采样规划可能偶发失败，复测成功不能抹去当次失败记录。

## 自动验收与日志

关闭已有仿真后，在 driver 目录执行：

```bash
source scripts/env.sh
python3 scripts/acceptance.py
```

脚本自动切换无挡板和挡板场景，执行 5 个注册测试，在 `.runtime/acceptance-时间/` 生成报告和日志。进程强杀、反馈断流、性能与速度对照属于单独专项测试，不包含在每次自动验收内。

driver 完整日志为 `.runtime/isaac-services.log`，Isaac 日志为 `.runtime/isaac-scene.log`；任务日志为各入口目录下的 `logs/<run_id>.log`。运行数据和构建产物不提交 Git。任务终端 `Ctrl+C` 请求取消任务；driver 终端 `Ctrl+C` 关闭本次会话。

## 更多说明

- [ROS 2 调用、任务锁与取消](piper_x_driver/docs/ROS_INTERFACES.md)
- [速度、加速度和配置](piper_x_driver/docs/CONFIGURATION.md)
- [运动契约与真机边界](piper_x_driver/docs/CONTRACT.md)

阶段记录用于追溯历史；当前用法以本 README 和上述接口文档为准。
