# 第一阶段：官方 MoveIt 模拟硬件演示

日期：2026-11-23。服务器：rl@100.73.76.90。
项目路径：/home/rl/workspace/piper_x_ws/piper_x_driver。

## 已完成

- 官方 ROS 仓库、嵌套 URDF 模型和 SDK 作为固定提交的 Git 子模块引入。
- ROS 2 Humble 下编译 4 个官方功能包通过。
- 使用系统 Python 3.10 的项目 venv，继承 ROS 系统依赖，SDK 安装在项目内，不修改 Conda。
- 修复服务器既有依赖：补装 ros-humble-moveit-ros-perception；Pilz 从 2.5.9 升至 2.5.10。
- 启动检查：OMPL、CHOMP、Pilz 三种规划插件均成功加载。
- PiPER-X + 官方夹爪，规划目标关节角 [0, 0.4, -0.5, 0, 0.1, 0] rad；OMPL 生成 22 个轨迹点。
- FollowJointTrajectory 返回成功；模拟关节反馈与规划轨迹终点误差为 0 rad。
- RViz 在 Xvfb 虚拟显示环境启动，模型加载、规划组和交互标记初始化成功。尚未人工验收远程桌面显示或验证 GPU 加速。
- 所有测试仅使用 mock 后端，未启动真机驱动；测试结束已发送退出信号。

## 使用

在干净的 Bash 终端中执行，不加载其他工作空间或 Conda：

```bash
cd /home/rl/workspace/piper_x_ws/piper_x_driver
bash scripts/build.sh
bash scripts/check_mock.sh
```

图形桌面终端启动 RViz：

```bash
cd /home/rl/workspace/piper_x_ws/piper_x_driver
bash scripts/run_mock.sh
```

项目默认 ROS_DOMAIN_ID=73、ROS_LOCALHOST_ONLY=1，仅限服务器本机通信；后续需要跨机通信时再调整。
`check_mock.sh` 启动和停止自己的无界面演示，并发送一条测试轨迹，只用于无真机的独立 mock 会话。
日志和 JSON 结果保存在 .runtime/，不提交 Git。

## 已知边界

- mock 的 0 rad 误差来自模拟硬件复制指令，不代表实际机械臂精度。
- 默认传感器配置没有实际点云输入，尚未测试感知避障。
- 虚拟显示 RViz 提示缺少 /recognize_objects 动作服务，当前未部署物体识别，不影响本阶段规划。
- 部分依赖存在弃用和非实时内核提示；尚未测量实时性能。
- MoveIt 在 SIGINT 退出时存在 class_loader 卸载警告和 -11；依赖修复后仍复现。运行期间规划与执行通过，但正常退出尚未通过，需后续定位。
- Isaac、接触抓取、CAN 和真机均未验收。

## 下一阶段

先验证远程图形访问，再用官方 PiPER-X Isaac 资产与配套配置接入物理仿真。保持官方代码不改；仅在实际验证发现缺口后增加配置或代码。
