"""PhysX shapes on the existing wrist rigid body, in the calibrated wrist frame."""
from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics

from gemini_shared_config import CONFIG


def _enable_shape(prim):
    UsdPhysics.CollisionAPI.Apply(prim)
    shape = PhysxSchema.PhysxCollisionAPI.Apply(prim)
    shape.CreateContactOffsetAttr(0.001)
    shape.CreateRestOffsetAttr(0.0)


def add_gemini_collisions(stage, attachment):
    path = attachment.GetPath()
    bracket = stage.GetPrimAtPath(path.AppendChild('Bracket'))
    UsdPhysics.MeshCollisionAPI.Apply(bracket).CreateApproximationAttr('convexDecomposition')
    _enable_shape(bracket)

    _, _, xyz, rpy = next(f for f in CONFIG['frames'] if f[0] == 'gemini_camera_link')
    center, size = CONFIG['camera_box']
    from gemini_sensor import _matrix
    mount = UsdGeom.Xform.Define(stage,path.AppendChild('CameraCollisionMount'))
    mount.AddTransformOp().Set(Gf.Matrix4d(*_matrix(xyz,rpy).T.ravel().tolist()))
    camera = UsdGeom.Cube.Define(stage,mount.GetPath().AppendChild('CameraCollider'))
    camera.CreateSizeAttr(1.)
    camera.AddTranslateOp().Set(Gf.Vec3d(*center))
    camera.AddScaleOp().Set(Gf.Vec3f(*size))
    camera.MakeInvisible()
    _enable_shape(camera.GetPrim())
    print('PIPER_GEMINI_PHYSICS', bracket.GetPath(),camera.GetPath(),flush=True)
    return bracket.GetPath(), camera.GetPath()
