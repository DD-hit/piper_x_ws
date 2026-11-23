# PiPER-X 空载运动任务

本项目作为 ROS 2 Humble 客户端独立运行。依赖系统安装的 `rclpy`、`tf2_ros`、`moveit_msgs`、`control_msgs` 及其标准消息依赖。无需导入驱动项目、Isaac Python 或厂商 SDK。

在服务器的 `piper_x_ws` 项目中运行 `PIPER_DEMO_BARRIER=1 bash scripts/run_isaac.sh`，等待服务日志显示 `MoveIt confirmed demo_barrier`。如分开启动物理场景和机械臂服务，两个终端都要设置 `PIPER_DEMO_BARRIER=1`。挡板尺寸与位置由服务项目的 `sim/isaac/barrier.json` 定义。

在本项目执行 `bash run_sim_demo.sh` 运行空载任务，或 `bash run_sim_demo.sh -p barrier_trial:=true` 同时验证挡板。程序等待关节状态、坐标变换及动作接口，执行预备姿态、TCP 上移、夹爪开合，并返回。挡板模式会检查两侧姿态的关节插值中点确实与挡板碰撞，再请求 MoveIt 规划并执行两侧之间的运动。

日志位于 `logs/`。失败会记录步骤、原因并以非零状态退出。已接收的动作超时会请求取消；通信中断时取消结果可能无法确认，应检查控制器状态。重跑前先核对现场状态，程序从最新反馈开始。请保持运动指令由单个任务来源控制。

业务接口包括 `move_action`、`gripper_controller/follow_joint_trajectory`、`joint_states`、TF 的 `base_link` 和 `tcp_link`。挡板验收还读取 `get_planning_scene`、`check_state_validity`。当前配置固定六个关节 `joint1` 至 `joint6`、规划组 `arm` 和以米表示总开度的 `gripper`。控制帧和规划组应与部署的模型一致。真机迁移需要校验工具坐标、控制器和动作反馈、限位与负载，切换 ROS 域与时钟；`run_sim_demo.sh` 专用于仿真。当前 `tcp_link` 为模型定义的工具帧，实际夹持点需要核对工具偏移。
