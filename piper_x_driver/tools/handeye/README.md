手眼标定工具适用于腕部固定相机。工具通过 ROS 2 读取彩色图、CameraInfo 和机械臂 TF，手动采样，离线求解。所有长度配置用米。

1. 在驱动根目录运行 `source scripts/env.sh`。Python 环境需要带 `aruco` 模块的 OpenCV、NumPy、SciPy 和 PyYAML。编辑 `tools/handeye/board.yaml`，填入板的横纵格数、实测格长、标记长、字典和 legacy 图案开关。图案开关要与实际使用的板一致。
2. 仿真中先生成标定板，并在启动 Isaac 时指定板面纹理。生成命令同时保存板规格。可通过 `PIPER_HANDEYE_BOARD_XYZ` 和 `PIPER_HANDEYE_BOARD_RPY_DEG` 调整板中心的位置和朝向。采样期间板必须固定。真机中直接固定同规格的打印板，核对 `hardware.yaml` 中的话题、光学坐标系和 ROS_DOMAIN_ID。
3. 在第二个终端启动采样。每换一个姿态，等腕部停稳，确认板清晰可见，再按回车。建议采约 30 组，包含绕不同轴的旋转。最后采集的 4 组默认留作验收，可用 `--holdout 8` 留出 8 组，选几个和训练组不同的姿态。输入 `q` 结束。相同配置可以对同一个样本文件续采。
4. 执行求解。YAML 中保存彩色光学坐标系在腕部坐标系下的平移和四元数，四元数顺序为 xyzw。验收误差是留出组求得的板位姿相对于训练组参考板位姿的差异。工具不会发布 TF，也不会覆盖已有结果。真机使用独立的样本文件和结果文件。

仿真启动命令，从驱动根目录执行：

```bash
source scripts/env.sh
python3 tools/handeye/handeye.py board --output .runtime/handeye/board.png
PIPER_HANDEYE_BOARD="$PWD/.runtime/handeye/board.png" \
  bash scripts/run_isaac.sh --stream --rviz wrist_attachment:=gemini336l
```

另一个终端采集并求解：

```bash
source scripts/env.sh
ROS_DOMAIN_ID=74 python3 tools/handeye/handeye.py capture \
  --profile tools/handeye/isaac.yaml --samples .runtime/handeye/isaac-samples.json
python3 tools/handeye/handeye.py solve \
  --samples .runtime/handeye/isaac-samples.json --output .runtime/handeye/isaac-result.yaml
```

换板时两个命令都传入相同的 `--board path/to/board.yaml`。生成的板图自带一格白边，按实际比例打印，校验方格尺寸后再采样。仿真外参可与共享模型中的 TF 对照，深度点云对齐要在接入规划地图前单独验证。


## 最近一次仿真标定与到点实验

使用 PiPER-X 腕部 Gemini 336L 彩色图像。采集了 32 组有效样本，24 组用于求解，8 组留出验证。相机到板中心的距离为 27.7–40.0 cm，板面倾角为 1.4–39.3°，图像内板面旋转跨度约 105°。

求解结果为彩色光学坐标系在 `link6` 下的位姿。变换把光学坐标系中的点转换到 `link6`。

```yaml
parent_frame: link6
child_frame: gemini_camera_color_optical_frame
transform_convention: parent_T_child
translation_m: [-0.023982917, -0.070167004, 0.042930424]
quaternion_xyzw: [-0.173774675, 0.000546859, -0.002027959, 0.984783200]
```

外参与仿真名义配置相差 0.64 mm 和 0.075°。平均重投影误差为 0.088 px。留出组在底座坐标系下的标定板位置 RMS 为 0.40 mm，最大值为 0.75 mm，姿态 RMS 为 0.14°。

另采 20 组未参与标定的图像。每组用同一彩色图像和腕部 TF，分别用两套外参换算带相同悬停偏移的目标点。两组从同一初始姿态执行 `MoveP`，按仿真目标真值计算位置误差。

| 外参 | 到点成功次数 | 平均坐标换算误差 | 平均 TCP 到点误差 | 最大 TCP 到点误差 |
| --- | ---: | ---: | ---: | ---: |
| CAD 名义外参 | 20/20 | 0.34 mm | 2.06 mm | 3.15 mm |
| 本轮手眼外参 | 20/20 | 0.81 mm | 2.08 mm | 3.28 mm |

该实验使用彩色图像估计板位姿，Isaac 渲染相机由名义外参配置。真机使用实际安装后的数据重新标定与验收。
