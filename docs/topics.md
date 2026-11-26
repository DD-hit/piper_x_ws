# ROS 2 topic and action contracts

## Robot state

- `/joint_states`: `sensor_msgs/msg/JointState`
- `/tf`, `/tf_static`: robot and sensor transforms
- `/piper_x/status`: `piper_x_interfaces/msg/ArmStatus`

## Camera contract

The camera provider owns image publication. Isaac Sim publishes the simulation stream. A vendor camera driver publishes the hardware stream. A launch-level remap or adapter provides the agreed names:

- `/piper_x/camera/color/image_raw`: `sensor_msgs/msg/Image`
- `/piper_x/camera/color/camera_info`: `sensor_msgs/msg/CameraInfo`

The motion driver does not subscribe to or interpret image content.

## Motion actions

- `/piper_x/move_j`
- `/piper_x/move_p`
- `/piper_x/move_l`
- `/piper_x/move_c`
- `/piper_x/move_t`
- `/piper_x/set_gripper`
- `/piper_x/run_task`

These names and message units remain stable across simulation and future hardware.
