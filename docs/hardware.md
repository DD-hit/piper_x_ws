# Future hardware launch contract

The hardware launch will compose:

- PiPER `pyAgxArm`/CAN backend.
- `robot_state_publisher` using the same robot description.
- The official camera vendor ROS 2 driver.
- A camera topic/frame remap if needed.
- A hardware-specific calibration profile from `piper_x_calibration/config/real/`.

The task layer should not know whether the image came from Isaac Sim or a physical camera. Hardware deployment must use a newly measured real-camera calibration profile. The simulation profile is not a substitute for real calibration.
