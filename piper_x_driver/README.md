# PiPER-X 仿真与真机工作空间

路径：`/home/rl/workspace/piper_x_ws/piper_x_driver`。
系统：Ubuntu 22.04 / ROS 2 Humble。官方代码通过 Git 子模块固定提交；不自动跟随上游更新。

## 项目内容

- `src/agx_arm_ros`：官方 ROS 2 驱动、MoveIt、模型及消息包。
- `vendor/pyAgxArm`：官方 Python SDK；使用隔离于 Conda 的项目环境。
- `scripts`：环境加载、编译和官方演示启动脚本。
- `docs`：阶段验证记录。
- `build`、`install`、`log`：编译产物，不提交。

## 阶段

1. 官方 MoveIt 模拟硬件演示：模型、规划和轨迹执行；无真机、无接触物理仿真。
2. Isaac 物理仿真：按官方资源接入，验证命令、反馈和仿真时间；本阶段尚未部署。
3. 真机：官方 SDK + CAN 驱动，使用实际反馈；本阶段尚未部署。

初期不创建自定义机器人驱动或规划器，不启动真机控制节点。
标准电动夹爪作为演示配置；实物末端型号待接真机前核实。

## 使用

```bash
cd /home/rl/workspace/piper_x_ws/piper_x_driver
bash scripts/build.sh
bash scripts/run_mock.sh
```

`run_mock.sh` 默认打开 RViz，需要在服务器图形桌面的终端中运行。
无图形启动：`bash scripts/run_mock.sh use_rviz:=false`。
不要在此运行其他工作空间的 setup；不要与同名控制话题的真机驱动同时运行。

首次获取项目代码需 `git submodule update --init --recursive`。
Isaac 仍使用其自身 Python；不要在本项目 Python 环境安装 Isaac。

自动验收：`bash scripts/check_mock.sh`。详见 `docs/stage1.md`。官方提交记录见 `docs/upstream_versions.json`。

## 第二阶段：Isaac 物理仿真

远程桌面终端：`bash scripts/run_isaac.sh --rviz`。
Isaac 在后台执行物理仿真，RViz 显示实测仿真关节；与第一阶段的模拟硬件不同。
默认选择 GPU 1、ROS 域 74；Ctrl+C 关闭本次会话。详情和边界见 `docs/stage2.md`。
