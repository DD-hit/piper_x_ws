# PiPER-X architecture

## Responsibility boundaries

- `piper_x_interfaces` defines stable ROS 2 actions and messages.
- `piper_x_description` owns robot links, limits, and MoveIt model inputs.
- `piper_x_control` owns motion semantics, state, cancellation, stopping, and faults.
- `piper_x_isaac_sim` is the Isaac Sim backend and simulated sensor integration. It does not perform perception or choose task actions.
- `piper_x_calibration` owns calibration artifacts and static calibration TF publication.
- `piper_x_tasks` owns perception, target geometry, grasp strategy, and acceptance.
- `agx_arm_ros` is upstream/vendor hardware integration.

## Runtime contracts

Simulation launch and future hardware launch must provide the same robot-facing contracts: `joint_states`, robot TF including `base_link -> link6`, camera `Image` and `CameraInfo` topics when a camera is deployed, and the `/piper_x/move_*`, `/piper_x/set_gripper`, and `/piper_x/status` interfaces. Isaac Sim publishes simulated camera data. A real camera vendor node publishes hardware data. Launch-level remaps or sensor adapters normalize names. The motion driver does not interpret image content.

## Calibration

Robot state supplies dynamic `base_link -> link6`. In Isaac, the nominal camera mount is part of the robot description and robot_state_publisher owns its TF. In deployments without this transform, a calibrated static publisher supplies it. Applications query TF2 and CameraInfo. Raw samples and validation reports are experiment records under the workspace `.runtime/handeye`, while validated runtime profiles live in `piper_x_calibration/config`.

## Migration

Build and scripts run from the workspace root. Tasks live in the workspace root under `piper_x_tasks/`. New code follows these boundaries. A future PiPER hardware backend implements the same motion contract without changing task code.
