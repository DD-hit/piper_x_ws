# Isaac 方块抓取与接触测量

在 driver 中执行 `bash scripts/build.sh`。本次修改了 `SetGripper.action`，运行任务的终端也应加载新构建的接口。

```bash
PIPER_DEMO_BARRIER=0 PIPER_GRASP_SCENE=1 bash scripts/run_isaac.sh --stream --rviz
```

等待 `ROS 2 task actions ready` 后，在另一个 driver 终端同步规划场景，再提交任务。

```bash
source scripts/env.sh
export ROS_DOMAIN_ID=74
python3 src/piper_x_isaac/runtime/load_grasp_scene.py
cd ../piper_x_tasks
bash run_isaac_task.sh run test.pick_and_place
```

每次重复前重启仿真并重新同步场景。`SetGripper` 返回 `REACHED_TARGET`、`BLOCKED_CLOSING` 或 `BLOCKED_OPENING`。当实际开度仍偏离目标，且持续收到反馈的 1 秒仿真时间内开度波动不超过 0.5 mm，判定为受阻。动作结束后保留原位置目标和驱动力上限，直到下一条夹爪命令。方块是否抓起仍按实际抬升和落点验收。

搬运使用 `MoveP.AVOID_OBSTACLES`，速度与加速度比例均为 0.1。方块的静摩擦系数为 1.3，动摩擦系数为 1.1。桌面材质和双指驱动力上限沿用原设置。

Isaac 按约 20 Hz 墙钟采集同帧的仿真时间、双指位置目标、实际开度、双指对方块的净法向接触力和物体相对夹爪的位置。原始数据为 `.runtime/grasp-telemetry.jsonl`，每次启动覆盖。各任务的分阶段记录在 `tests/pick_and_place/logs/<run_id>-grasp.json`。当前方案的重复验收数据保存在 `.runtime/grasp-friction-repeat/`。

双指 `10 N` 是位置驱动配置上限。采样接触力是 PhysX 对方块的净法向力模长；不代表真机夹持力或连续时间峰值。场景固定为单个已知位置的 30 mm 方块，摩擦、载荷和速度结果不外推至其他工件。MoveIt 的附着物用于规划；Isaac 中方块依靠碰撞与摩擦搬运。
