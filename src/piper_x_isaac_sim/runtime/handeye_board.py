"""Load a generated ChArUco texture onto a metric plane in Isaac Sim."""
import json
import os
from pathlib import Path

from pxr import Gf, Sdf, UsdGeom, UsdShade


def add_handeye_board(stage, texture_file):
    texture = Path(texture_file).resolve(strict=True)
    spec = json.loads(texture.with_suffix('.json').read_text())
    width = (spec['squares_x'] + 2) * spec['square_m']
    height = (spec['squares_y'] + 2) * spec['square_m']
    x, y = width / 2, height / 2
    root = '/World/HandeyeBoard'
    mesh = UsdGeom.Mesh.Define(stage, root)
    corner_offset = (spec["square_m"], spec["square_m"])
    mesh.CreatePointsAttr([(-x+corner_offset[0],-y+corner_offset[1],0),
                           (x+corner_offset[0],-y+corner_offset[1],0),
                           (x+corner_offset[0],y+corner_offset[1],0),
                           (-x+corner_offset[0],y+corner_offset[1],0)])
    mesh.CreateFaceVertexCountsAttr([4])
    mesh.CreateFaceVertexIndicesAttr([0,1,2,3])
    mesh.CreateSubdivisionSchemeAttr('none')
    mesh.CreateDoubleSidedAttr(True)
    uv = UsdGeom.PrimvarsAPI(mesh).CreatePrimvar('st',Sdf.ValueTypeNames.TexCoord2fArray,UsdGeom.Tokens.vertex)
    uv.Set([(0,0),(1,0),(1,1),(0,1)])
    placement = UsdGeom.XformCommonAPI(mesh)
    xyz = [float(v) for v in os.environ.get('PIPER_HANDEYE_BOARD_XYZ','0.48 0 0.24').split()]
    rpy = [float(v) for v in os.environ.get('PIPER_HANDEYE_BOARD_RPY_DEG','0 -90 0').split()]
    placement.SetTranslate(Gf.Vec3d(*xyz))
    placement.SetRotate(Gf.Vec3f(*rpy),UsdGeom.XformCommonAPI.RotationOrderXYZ)
    material = UsdShade.Material.Define(stage,root+'/Material')
    surface = UsdShade.Shader.Define(stage,root+'/Material/Surface')
    surface.CreateIdAttr('UsdPreviewSurface')
    surface.CreateInput('roughness',Sdf.ValueTypeNames.Float).Set(1.)
    reader = UsdShade.Shader.Define(stage,root+'/Material/UV')
    reader.CreateIdAttr('UsdPrimvarReader_float2')
    reader.CreateInput('varname',Sdf.ValueTypeNames.Token).Set('st')
    tex = UsdShade.Shader.Define(stage,root+'/Material/Texture')
    tex.CreateIdAttr('UsdUVTexture')
    tex.CreateInput('file',Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath(str(texture)))
    tex.CreateInput('st',Sdf.ValueTypeNames.Float2).ConnectToSource(
        reader.CreateOutput('result',Sdf.ValueTypeNames.Float2))
    color = tex.CreateOutput('rgb',Sdf.ValueTypeNames.Float3)
    surface.CreateInput('emissiveColor',Sdf.ValueTypeNames.Color3f).ConnectToSource(color)
    material.CreateSurfaceOutput().ConnectToSource(surface.CreateOutput('surface',Sdf.ValueTypeNames.Token))
    UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(material)
    print('PIPER_HANDEYE_BOARD',texture,xyz,rpy,flush=True)
