# Driver 配置

启动时读取 `config/driver.yaml`，或用环境变量 `PIPER_DRIVER_CONFIG` 指向完整配置文件。配置在启动时生效；修改后重启 driver。它不是任务参数文件，任务目标和验收门槛仍由独立任务管理。

MoveJ/P/L/C/T 都接收 `velocity_scale` 与 `acceleration_scale`。零表示使用配置默认值；显式值必须位于 (0, 配置最大值]，NaN、无穷和越界请求拒绝。比例相对各关节模型限制，不是 TCP 的 m/s 或 m/s²。

MoveIt 避障规划分别使用两个比例。笛卡尔路径在已有时间参数化轨迹上统一拉长时间，以同时满足速度与加速度缩放，并检查轨迹点携带的模型限值；此方法不缩短原始轨迹，也不意味着两个比例必然被用满。直接关节运动用零首末速度三次插值，时长同时满足解析峰值速度和加速度约束。命令参考限值不是实测电机加速度的保证。

关节位置与速度从生成的官方 URDF 读取。加速度取当前官方 `joint_limits.yaml`，当前是仿真模型配置，未视为真机标定。夹爪尚未提供独立速度和加速度请求。

YAML 当前集中管理：两个默认比例和最大比例、规划时间与次数、笛卡尔步长、关节和 TCP 到位容差、等待稳定期限、反馈超时、下游接口与目标确认期限、执行等待余量、取消与停止确认期限、状态发布频率、夹爪开度范围及误差。

规划组、坐标系、接口名称、IK 时限、圆弧最大采样点数等仍有固定约定。没有伪装成已可配置；真机 CAN、工具标定、驱动模式需要后续后端配置。现有独立反馈看门狗仍使用 5 秒服务退出期限，动作级反馈期限由此 YAML 控制。

示例：`ros2 action send_goal /piper_x/move_t piper_x_interfaces/action/MoveT '{translation: {header: {frame_id: base_link}, vector: {z: 0.01}}, velocity_scale: 0.2, acceleration_scale: 0.1}' --feedback`。
