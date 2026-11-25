仅用于 Isaac Sim。用 PIPER_GEMINI=1 启动驱动，保持场景为空并停止其他任务。
从任务目录运行：

    source ../piper_x_driver/scripts/env.sh
    ROS_DOMAIN_ID=74 PIPER_GEMINI_TEST_PAUSE=5 python3 tests/gemini_movep/main.py

每组先用 MoveJ 移到起点，再添加障碍物。脚本确认起点和终点均无碰撞，但关节直插中点只与对应的相机或安装板发生碰撞。随后保持障碍物，用 MoveP 避障模式到达目标，并检查实际执行的关节反馈采样。脚本在每组结束后移除测试障碍物。

挡板在 MoveIt Planning Scene 中参与规划，同时镜像为 Isaac Sim 中的橙色视觉模型。可在 RViz 的 MotionPlanning 场景或 Isaac Sim 串流里观察。默认暂停 5 秒。
