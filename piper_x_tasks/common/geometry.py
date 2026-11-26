import numpy as np
from scipy.spatial.transform import Rotation

def transform_from_tf(tf):
    q=tf.transform.rotation
    t=tf.transform.translation
    r=Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix()
    out=np.eye(4); out[:3,:3]=r; out[:3,3]=[t.x,t.y,t.z]; return out

def pixel_to_plane(u,v,k,t_base_camera,z_plane):
    k=np.asarray(k,float); ray_cam=np.linalg.solve(k,[float(u),float(v),1.0])
    ray_base=t_base_camera[:3,:3]@ray_cam; origin=t_base_camera[:3,3]
    if abs(ray_base[2])<1e-9: raise ValueError('camera ray parallel to plane')
    scale=(float(z_plane)-origin[2])/ray_base[2]
    if scale<=0: raise ValueError('plane is behind camera ray')
    return origin+scale*ray_base
