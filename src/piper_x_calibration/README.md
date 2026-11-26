# piper_x_calibration

Owns calibration artifacts and static calibration TF publication. It does not
implement robot motion, camera drivers, perception, or task logic.

The simulation profile contains the validated Isaac Sim hand-eye result. Keep
raw samples and reports under `.runtime/handeye/` as experiment records. Create
a separate hardware profile after the real camera is installed and calibrated.
