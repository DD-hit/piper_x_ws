"""Mirror the local MoveP test fixture into Isaac's viewport."""
import json
import time
from pathlib import Path

from pxr import Gf, UsdGeom


class GeminiFixtureVisual:
    def __init__(self, stage):
        self.stage = stage
        self.file = Path('.runtime/gemini-movep-fixture.json')
        self.prim_path = '/World/GeminiMovePTestObstacle'
        self.previous = None
        self.next_read = 0.

    def update(self):
        now = time.monotonic()
        if now < self.next_read:
            return
        self.next_read = now + 0.1
        try:
            data = json.loads(self.file.read_text())
        except (FileNotFoundError, ValueError):
            data = None
        if data and data['expires'] < time.time():
            data = None
        if data == self.previous:
            return
        self.previous = data
        self.stage.RemovePrim(self.prim_path)
        if not data:
            return
        cube = UsdGeom.Cube.Define(self.stage, self.prim_path)
        cube.CreateSizeAttr(1.)
        cube.CreateDisplayColorAttr([Gf.Vec3f(1., .32, .05)])
        cube.AddTranslateOp().Set(Gf.Vec3d(*data['position']))
        x, y, z, w = data['quaternion']
        cube.AddOrientOp().Set(Gf.Quatf(w, Gf.Vec3f(x, y, z)))
        cube.AddScaleOp().Set(Gf.Vec3f(*data['dimensions']))
        from isaacsim.core.utils.viewports import set_camera_view
        target = Gf.Vec3d(*data['position'])
        eye = target + Gf.Vec3d(.30, -.27, .19)
        set_camera_view(eye=tuple(eye), target=tuple(target))
        print('PIPER_GEMINI_TEST_OBSTACLE', data['position'], flush=True)
