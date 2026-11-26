# Backend boundaries

## Common contract

Every robot backend must provide joint state, TCP state, gripper state, enable/disable, motion execution, cancellation, stop confirmation, and fault reporting through the existing driver interfaces.

## Isaac Sim

`piper_x_isaac_sim` owns the Isaac scene, simulated robot controller, and simulator sensor bridge. It publishes simulated camera data through Isaac integration and delegates motion semantics to the driver.

## Hardware

The future hardware backend owns `pyAgxArm`, CAN transport, state polling, trajectory writes, stop confirmation, recovery, and vendor error conversion. It must not change task APIs or write camera topics.

## Tasks

Tasks consume camera messages, CameraInfo, TF2, and motion actions. They own perception, pixel geometry, grasp strategy, and acceptance checks.
