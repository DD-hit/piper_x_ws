在现有 Isaac 与 MoveIt 会话退出后，从驱动目录运行：

    bash scripts/run_isaac.sh --stream --rviz wrist_attachment:=gemini336l

此开关同时启用 Gemini 外观、彩色与深度渲染以及 MoveIt 碰撞模型。彩色、深度和各自 CameraInfo 位于 /piper_x/camera。robot_state_publisher 根据共享包中的 config/gemini_frames.json 发布内部名义外参。该配置沿用奥比中光 v2-main 的 gemini_335_L_336_L Xacro，安装位姿取自两固定孔配准，光学内参为仿真参数。

需要运行验证时，另开终端并确保当前规划场景为空、机械臂静止：

    source scripts/env.sh
    ROS_DOMAIN_ID=74 python3 scripts/verify_gemini.py

验证会临时添加一个小挡板，检查它只与相机发生碰撞，然后移除。启用 Gemini 后，相机和支架同时具有 MoveIt 规划碰撞几何和 Isaac Sim 接触碰撞体。Isaac 将相机近似为包围盒，将支架网格做凸分解，两者共享现有腕部刚体。未做附件负载质量、惯量或真机接触参数标定。MoveP 绕障脚本的橙色挡板是规划障碍物的视觉镜像。真机内外参需要用官方驱动复核。

共享模型位于 src/piper_x_description。构建后用 ros2 pkg prefix --share piper_x_description 查询资源目录。
Isaac USD 保留在 src/piper_x_isaac/runtime。旧启动开关 PIPER_GEMINI=1 仍支持。
真机启动可复用 robot_description.py 的 with_wrist_attachment，并传 include_sensor_frames=False，由 Orbbec 驱动发布相机内部 TF。接线时还需确认驱动的根坐标系与 gemini_camera_link 的对应关系。
