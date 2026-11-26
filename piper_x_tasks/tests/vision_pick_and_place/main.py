import json
import os
import runpy
import sys
import time
from pathlib import Path
TASKS_ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(TASKS_ROOT))
import numpy as np
import rclpy
from scipy.optimize import least_squares
from itertools import product
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from tf2_ros import Buffer, TransformListener
from piper_x_tasks.common.image import image_to_bgr
from piper_x_tasks.common.perception import detect_color
from piper_x_tasks.common.geometry import transform_from_tf, pixel_to_plane

IMAGE = "/piper_x/camera/color/image_raw"
INFO = "/piper_x/camera/color/camera_info"
CAMERA = "gemini_camera_color_optical_frame"

def locate(arm):
    telemetry_path = TASKS_ROOT / ".runtime/grasp-state.json"
    if not telemetry_path.exists() or time.time() - json.loads(telemetry_path.read_text())["wall_time"] > 2.0:
        raise RuntimeError("Isaac grasp scene is unavailable or its telemetry is stale")
    images, infos = [], []
    arm.node.create_subscription(Image, IMAGE, images.append, qos_profile_sensor_data)
    arm.node.create_subscription(CameraInfo, INFO, infos.append, qos_profile_sensor_data)
    tf_buffer = Buffer()
    TransformListener(tf_buffer, arm.node)
    deadline = time.monotonic() + 15.0
    while (not images or not infos) and time.monotonic() < deadline:
        rclpy.spin_once(arm.node, timeout_sec=0.1)
    if not images or not infos:
        raise RuntimeError("camera image or CameraInfo unavailable")
    detection, _ = detect_color(image_to_bgr(images[-1]), lower=(0, 120, 80), upper=(25, 255, 255), min_area=100)
    if detection is None:
        raise RuntimeError("no orange cube detected")
    stamp = rclpy.time.Time.from_msg(images[-1].header.stamp)
    while not tf_buffer.can_transform("base_link", CAMERA, stamp) and time.monotonic() < deadline:
        rclpy.spin_once(arm.node, timeout_sec=0.1)
    transform = tf_buffer.lookup_transform("base_link", CAMERA, stamp)
    k = np.asarray(infos[-1].k, dtype=float).reshape(3, 3)
    # Fit the known 30 mm cube to its image bounds. The silhouette includes
    # multiple faces, so its centroid does not lie on the top face.
    base_from_camera = transform_from_tf(transform)
    camera_from_base = np.linalg.inv(base_from_camera)
    center_z = 0.095
    seed = pixel_to_plane(*detection["center_uv"], k, base_from_camera, center_z)
    corners = np.array(list(product((-0.015, 0.015), repeat=3)))
    u, v, width, height = detection["box_xywh"]
    observed = np.array([u, v, u + width - 1, v + height - 1])

    def residual(xy):
        vertices = corners + [*xy, center_z]
        points = vertices @ camera_from_base[:3, :3].T + camera_from_base[:3, 3]
        pixels = points @ k.T
        uv = pixels[:, :2] / pixels[:, 2:]
        return np.r_[uv.min(axis=0), uv.max(axis=0)] - observed

    fit = least_squares(residual, seed[:2])
    if not fit.success or not np.isfinite(fit.x).all():
        raise RuntimeError(f"Cube fit failed: {fit.message}")
    x, y = map(float, fit.x)
    report = {
        "detection": detection,
        "target_base_xy": [x, y],
        "box_edge_errors_px": fit.fun.tolist(),
    }
    run_id = os.environ["PIPER_RUN_ID"]
    report_path = Path("logs") / f"{run_id}-vision-pick.json"
    report_path.parent.mkdir(parents=True, exist_ok=True)
    truth = json.loads(telemetry_path.read_text())["position"]
    report["position_error_m"] = float(np.linalg.norm(fit.x - truth[:2]))
    report_path.write_text(json.dumps(report, indent=2))
    print(json.dumps(report), flush=True)
    if report["position_error_m"] > 0.012:
        raise RuntimeError("Visual target exceeds the simulation acceptance tolerance")
    return x, y


runpy.run_path(
    str(Path(__file__).resolve().parents[1] / "pick_and_place/main.py"),
    init_globals={"locate_pick_target": locate},
)
