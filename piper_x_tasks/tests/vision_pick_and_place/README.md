# Isaac 视觉抓取验收

在驱动目录运行 `PIPER_GRASP_SCENE=1 bash scripts/run_isaac.sh wrist_attachment:=gemini336l`。
服务就绪后，在任务目录运行 `bash run_isaac_task.sh run test.vision_pick_and_place`。

任务用 OpenCV 检测橙色方块，通过图像时间戳查询 TF2，拟合已知尺寸方块的图像边界。
仿真真值用于验证定位、抬升和最终落点。每次测试前需重置仿真场景。
