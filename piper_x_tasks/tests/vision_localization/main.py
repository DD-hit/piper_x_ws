import json, os, time
import sys
from pathlib import Path
TASKS_ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(TASKS_ROOT))
from pathlib import Path
import cv2, numpy as np, rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from tf2_ros import Buffer, TransformListener
from piper_x_driver.native_client import Arm
from piper_x_tasks.common.image import image_to_bgr
from piper_x_tasks.common.perception import detect_color
from piper_x_tasks.common.geometry import transform_from_tf,pixel_to_plane

IMAGE='/piper_x/camera/color/image_raw'; INFO='/piper_x/camera/color/camera_info'; CAMERA='gemini_camera_color_optical_frame'
with Arm() as arm:
    images=[]; infos=[]
    arm.node.create_subscription(Image,IMAGE,lambda m: images.append(m),qos_profile_sensor_data)
    arm.node.create_subscription(CameraInfo,INFO,lambda m: infos.append(m),qos_profile_sensor_data)
    tf=Buffer(); TransformListener(tf,arm.node)
    deadline=time.monotonic()+15
    while (not images or not infos) and time.monotonic()<deadline: rclpy.spin_once(arm.node,timeout_sec=.1)
    if not images or not infos: raise RuntimeError('camera image or CameraInfo unavailable')
    image=image_to_bgr(images[-1]); detection,mask=detect_color(image)
    if detection is None: raise RuntimeError('no blue target detected')
    info=infos[-1]; k=np.asarray(info.k,float).reshape(3,3)
    tr=tf.lookup_transform('base_link',CAMERA,rclpy.time.Time())
    xyz=pixel_to_plane(*detection['center_uv'],k,transform_from_tf(tr),float(os.getenv('PIPER_TABLE_Z','0.0')))
    out={'detected':True,'center_uv':detection['center_uv'],'area_px':detection['area_px'],'estimated_base_xyz_m':xyz.tolist(),'camera_frame':CAMERA}
    path=Path(os.environ.get('PIPER_TASKS_ROOT',Path.cwd()))/'logs'/f"{os.environ.get('PIPER_RUN_ID','local')}-vision-localization.json"; path.parent.mkdir(parents=True,exist_ok=True); path.write_text(json.dumps(out,indent=2)); print(json.dumps(out,indent=2),flush=True)
