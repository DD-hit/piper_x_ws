# 第二阶段：Isaac 物理仿真接入

日期：2026-11-23。系统：Ubuntu 22.04 / ROS 2 Humble / Isaac Sim 5.0.0-rc.45。

## 结果

- Isaac 无界面启动并执行物理步进；已导出并查看场景截图。
- ROS 发布仿真时钟和实际物理关节状态，命令经 TopicBasedSystem 到达 Isaac articulation。
- 当前官方 PiPER-X 模型与 MoveIt 同源。URDF 的 check_urdf 检查通过；本地镜像使用 cad urdf validate 校验通过，无问题（13 links、12 joints、8 movable、22 meshes）。镜像仅改变资源路径。
- OMPL 规划并生成 22 个轨迹点；FollowJointTrajectory 执行成功。
- 验收读取 Isaac 物理状态，与规划终点最大关节误差约 0.00469 rad，低于本阶段 0.03 rad 门槛。
- 夹爪目标开度 0.04 m 时，两指约 +0.0201/-0.0199 m；闭合约 +0.0001/0 m，均通过每指 0.003 m 门槛。
- 未验证物体抓取、相机、手眼标定、动态避障、全工作空间或真机。

## 为什么不直接使用旧 USD

服务器已有 piper_isaac_sim 旧 USD 与当前 agx_arm_urdf 的部分关节坐标、长度不同。初步链路测试后弃用旧资产；最终以当前官方 xacro 为共同来源，不混用版本。

scripts/prepare_isaac_model.py 展开官方 xacro，为 Isaac 导入生成临时 URDF。六个机械臂关节的原点、轴、限位、网格和惯量保留；移除 ROS 控制元数据和无几何的 gripper 聚合关节/空 link；将两指作为独立物理关节。
映射严格来自官方 mimic：joint1=q/2、joint2=-q/2，反馈总开度为 q1-q2。这里 joint1/joint2 指 gripper_joint1/gripper_joint2。
夹指位置驱动为仿真参数：stiffness=2000 N/m、damping=20 N·s/m，保留官方每指 10 N effort 上限。这些参数仅用于仿真，不代表实物标定。

## 文件

- scripts/run_isaac.sh：一键启动及会话退出，默认 Isaac 无界面，--rviz 打开 RViz。
- scripts/run_isaac_scene.sh：隔离 Isaac Python 3.11 与 ROS 系统 Python 3.10。
- src/piper_x_isaac/runtime/run_scene.py：官方 URDF 导入、固定基座、地面、光源、ROS 图。
- src/piper_x_isaac/runtime/moveit.launch.py：复用官方 MoveIt 配置，按官方仿真示例切换 TopicBasedSystem；use_sim_time=true。
- src/piper_x_isaac/runtime/joint_mapping.py：仅处理关节命名与官方夹指 mimic 映射，不实现规划或底层驱动。

官方源码未改。新增内容为官方资源的部署配置和必要的 Isaac 连接脚本。

## 启动

远程桌面终端（先用 Ctrl+C 关闭此前 mock 演示）：

```bash
cd /home/rl/workspace/piper_x_ws/piper_x_driver
bash scripts/run_isaac.sh --rviz
```

Isaac 在后台无界面执行物理仿真，RViz 显示其实际反馈；RViz 是显示窗口，不承担物理计算。
默认仅服务器本机 ROS 通信，域 74。mock 阶段域 73；分开域以避免旧会话干扰。
如果启动提示 session already running，请先退出已有的本项目 Isaac 会话，不要重复启动。
Ctrl+C 关闭本次进程组，不使用全局 pkill。

## Mac 实时观看与远程桌面控制

在 Windows App 的远程桌面终端中运行：

```bash
cd /home/rl/workspace/piper_x_ws/piper_x_driver
bash scripts/run_isaac.sh --stream --rviz
```

等终端显示 Isaac + MoveIt 已启动后，在 Mac 的 Isaac Sim WebRTC Streaming Client 中输入 `100.73.76.90` 并点击 Connect。Mac 显示 Isaac 的物理仿真画面，Windows App 中的 RViz 负责发送 MoveIt 运动请求。串流使用 TCP `49100` 和 UDP `47998`，Mac 与服务器通过 Tailscale 连通。

本机 Isaac Sim 5.0.0-rc 安装的串流依赖已放在项目的 `.deps/isaac`。若需要在新机器上重建，使用 Isaac 自带的 Python：

```bash
bash scripts/install_isaac_deps.sh
```

另一终端自动验收：

```bash
cd /home/rl/workspace/piper_x_ws/piper_x_driver
source scripts/env.sh
export ROS_DOMAIN_ID=74
python3 scripts/verify_mock.py --backend isaac
python3 scripts/verify_isaac_gripper.py
```

参数 --backend isaac 明确选择物理反馈，不能把 mock 验收结果用于本阶段。

## 日志与截图

日志在 .runtime/isaac-scene.log；执行结果在 .runtime/isaac_verification.json、isaac_gripper_verification.json。
会话运行时 touch .runtime/capture.request，可请求渲染截图到 .runtime/isaac_scene.png。
已验证无界面 GPU 渲染；未保证 xrdp 桌面直接运行 Isaac GUI 的 Vulkan 加速。

## 限制

导入默认的碰撞近似与驱动参数尚未针对真实抓取标定；只证明本次机械臂运动与空载夹爪开合。当前地面场景未加入桌面和物体。
Isaac 当前为候选版本，存在弃用和无窗口相关警告。尚未完成退出异常的独立审查。
