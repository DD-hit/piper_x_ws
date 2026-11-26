"""Render nominal color and depth streams from the Orbbec optical frames."""
import math
import numpy as np
import omni.graph.core as og
import omni.replicator.core as rep
from pxr import Gf, UsdGeom

from gemini_shared_config import CONFIG


def _matrix(xyz, rpy):
    r,p,y = rpy
    cr,sr,cp,sp,cy,sy = math.cos(r),math.sin(r),math.cos(p),math.sin(p),math.cos(y),math.sin(y)
    return np.array([[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr,xyz[0]],
                     [sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr,xyz[1]],
                     [-sp,cp*sr,cp*cr,xyz[2]],[0,0,0,1]])


def add_gemini_sensors(stage, attachment):
    transforms={'link6':np.eye(4)}
    for name,parent,xyz,rpy in CONFIG['frames']:
        transforms[name]=transforms[parent]@_matrix(xyz,rpy)
    keys=og.Controller.Keys
    nodes=[('Tick','omni.graph.action.OnPlaybackTick')]
    connections=[]
    values=[]
    products=[]
    for stream,kind,topic in [('color','rgb','image_raw'),('depth','depth','image_rect_raw')]:
        frame=f'gemini_camera_{stream}_optical_frame'
        camera=UsdGeom.Camera.Define(stage,attachment.GetPath().AppendChild(stream.title()+'Sensor'))
        # USD camera coordinates use -Z forward and +Y up. The ROS optical frame
        # uses +Z forward and +Y down. Convert only the camera prim orientation.
        optical_to_usd = np.diag([1., -1., -1., 1.])
        matrix = transforms[frame] @ optical_to_usd
        camera.AddTransformOp().Set(Gf.Matrix4d(*matrix.T.ravel().tolist()))
        camera.CreateFocalLengthAttr(10.)
        camera.CreateHorizontalApertureAttr(20.)
        camera.CreateVerticalApertureAttr(15.)
        camera.CreateClippingRangeAttr(Gf.Vec2f(.15,5.))
        product=rep.create.render_product(str(camera.GetPath()),(640,480))
        products.append(product)
        for suffix,helper,leaf in [('Image','ROS2CameraHelper',topic),('Info','ROS2CameraInfoHelper','camera_info')]:
            name=stream.title()+suffix
            nodes.append((name,'isaacsim.ros2.bridge.'+helper))
            connections.append(('Tick.outputs:tick',f'{name}.inputs:execIn'))
            for prop,val in [('frameId',frame),('topicName',f'/piper_x/camera/{stream}/{leaf}'),
                             ('renderProductPath',product.path),('frameSkipCount',3)]:
                values.append((f'{name}.inputs:{prop}',val))
            if suffix=='Image':
                values.append((f'{name}.inputs:type',kind))
    og.Controller.edit({'graph_path':'/World/GeminiROS','evaluator_name':'execution'},
                       {keys.CREATE_NODES:nodes,keys.CONNECT:connections,keys.SET_VALUES:values})
    print('PIPER_GEMINI_SENSORS_READY',flush=True)
    return products
