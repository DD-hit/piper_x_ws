# PiPER-X 腕部 RGB-D 相机

安装变换取自 AgileX `piper_isaac_sim` 的 `piper_x_description_d435.urdf`（提交 `8e1f88f`），应用在本项目当前模型的 `link6`。Isaac 使用理想针孔相机，RGB 和深度共用一个光学坐标系。分辨率为 640 × 480，焦距由仿真配置计算。此配置只导入安装变换，D435 外壳、质量、碰撞几何和双目噪声尚未加入。

```bash
PIPER_WRIST_CAMERA=1 PIPER_GRASP_SCENE=1 bash scripts/run_isaac.sh --stream --rviz
```

在另一个服务器终端从 driver 目录执行：

```bash
source scripts/env.sh
export ROS_DOMAIN_ID=74
ros2 topic list -t | grep /piper_x/camera
ros2 run tf2_ros tf2_echo base_link piper_wrist_camera_optical_frame
```

相机话题前缀为 `/piper_x/camera`。`color/image_raw` 为 `rgb8`，`depth/image_rect_raw` 为 `32FC1`（米），对应内参为 `color/camera_info` 和 `depth/camera_info`。订阅图像时可使用传感器 QoS。在 RViz 添加 Image 显示并选择彩色话题即可看腕部画面。光学坐标系通过 TF 随 `link6` 运动。当前抓取任务继续使用已知物体位置。
