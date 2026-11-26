# 坐标系与标定

- `base_link → link6` 由机器人状态和 `robot_state_publisher` 提供。
- Isaac 默认使用 `piper_x_description/config/gemini_frames.json` 的名义安装参数。
  同一份配置驱动渲染相机和 URDF 固定关节，由 `robot_state_publisher` 发布相机 TF。
- `piper_x_calibration` 保存手眼估计结果及来源。独立静态 TF 入口适用于
  对应光学子坐标系尚无发布者的部署。真机接入时选择对应实测标定配置。
- 业务按图像时间戳查询 `base_link` 到 `Image.header.frame_id` 的变换，
  并从 `CameraInfo` 读取内参。
