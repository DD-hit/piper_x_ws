import math

import omni.graph.core as og
import omni.replicator.core as rep
from pxr import Gf, UsdGeom, UsdPhysics


def add_wrist_camera(stage):
    parent, = (prim for prim in stage.Traverse()
               if prim.GetName() == 'link6' and prim.HasAPI(UsdPhysics.RigidBodyAPI))
    mount = UsdGeom.Xform.Define(stage, parent.GetPath().AppendChild('D435Mount'))
    xform = UsdGeom.XformCommonAPI(mount)
    xform.SetTranslate(Gf.Vec3d(0.029, -0.069, 0.022))
    xform.SetRotate((0., math.degrees(-1.22), math.degrees(1.57)), UsdGeom.XformCommonAPI.RotationOrderXYZ)
    camera = UsdGeom.Camera.Define(stage, mount.GetPath().AppendChild('D435Camera'))
    camera_xform = UsdGeom.XformCommonAPI(camera)
    camera_xform.SetTranslate(Gf.Vec3d(0.0106, 0.0175, 0.0125))
    camera_xform.SetRotate((90., 0., -90.), UsdGeom.XformCommonAPI.RotationOrderXYZ)
    camera.GetHorizontalApertureAttr().Set(20.)
    camera.GetVerticalApertureAttr().Set(15.)
    camera.GetFocalLengthAttr().Set(12.)
    camera.GetClippingRangeAttr().Set(Gf.Vec2f(0.05, 5.))
    product = rep.create.render_product(str(camera.GetPath()), (640, 480))
    keys = og.Controller.Keys
    nodes = [('Tick', 'omni.graph.action.OnPlaybackTick')]
    connections = []
    values = []
    for name, topic, kind in [('Color', 'color/image_raw', 'rgb'),
                              ('Depth', 'depth/image_rect_raw', 'depth'),
                              ('Info', 'color/camera_info', None),
                              ('DepthInfo', 'depth/camera_info', None)]:
        helper = 'ROS2CameraInfoHelper' if kind is None else 'ROS2CameraHelper'
        nodes.append((name, 'isaacsim.ros2.bridge.' + helper))
        connections.append(('Tick.outputs:tick', f'{name}.inputs:execIn'))
        values.extend([(f'{name}.inputs:frameId', 'piper_wrist_camera_optical_frame'),
                       (f'{name}.inputs:topicName', '/piper_x/camera/' + topic),
                       (f'{name}.inputs:renderProductPath', product.path),
                       (f'{name}.inputs:frameSkipCount', 3)])
        if kind:
            values.append((f'{name}.inputs:type', kind))
    og.Controller.edit({'graph_path': '/World/PiperWristCamera', 'evaluator_name': 'execution'},
                       {keys.CREATE_NODES: nodes, keys.CONNECT: connections, keys.SET_VALUES: values})
    return product
