"""Visual-only wrist attachment authored in the official link6 frame."""
from pathlib import Path

from pxr import UsdGeom, UsdPhysics


def add_gemini_visual(stage):
    wrist, = (prim for prim in stage.Traverse()
              if prim.GetName() == 'link6' and prim.HasAPI(UsdPhysics.RigidBodyAPI))
    attachment = UsdGeom.Xform.Define(stage, wrist.GetPath().AppendChild('Gemini336L'))
    asset = Path(__file__).with_name('gemini336l_wrist.usdc')
    if not attachment.GetPrim().GetReferences().AddReference(str(asset)):
        raise RuntimeError(f'Cannot reference {asset}')
    for name in ('Bracket', 'CameraHousing', 'FrontPanel', 'OpticalWindows'):
        if not stage.GetPrimAtPath(attachment.GetPath().AppendChild(name)).IsA(UsdGeom.Mesh):
            raise RuntimeError(f'Missing Gemini visual mesh: {name}')
    print('PIPER_GEMINI_VISUAL', attachment.GetPath(), flush=True)
    return attachment
