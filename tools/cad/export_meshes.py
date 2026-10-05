"""Export the CAD parts as STL meshes for the app's 3D view, plus the joint data needed to
animate them (app/assets/meshes/meshes.json). Run from the repository root with FreeCAD:

    "C:/Program Files/FreeCAD 1.0/bin/freecadcmd.exe" -c "exec(open('tools/cad/export_meshes.py').read())"

Meshes stay in the CAD frame (millimetres) in the as-modelled pose; the viewer moves each
part by the rigid transform from its CAD pose to its simulated pose.
"""
import json
import math
import os

import FreeCAD as App
import Import
import Mesh
import MeshPart
import Part

ROOT = os.getcwd()
OUT = os.path.join(ROOT, "app", "assets", "meshes")
os.makedirs(OUT, exist_ok=True)

doc = App.newDocument("bbp_meshes")
Import.insert(os.path.join(ROOT, "CAD", "FullASSY.step"), doc.Name)


def gshape(o):
    sh = o.Shape.copy()
    sh.Placement = o.getGlobalPlacement()
    return sh


def parts(prefix):
    return [o for o in doc.Objects if o.TypeId == "Part::Feature" and o.Label.startswith(prefix)]


def export(shape, name, deflection=0.15):
    m = MeshPart.meshFromShape(Shape=shape, LinearDeflection=deflection, AngularDeflection=0.35, Relative=False)
    m.write(os.path.join(OUT, name))
    return m.CountFacets


def cylinders(sh, radius, tol=0.02):
    return [(f.Surface.Center, f.Surface.Axis.normalize(), f.Area) for f in sh.Faces
            if type(f.Surface).__name__ == "Cylinder" and abs(f.Surface.Radius - radius) < tol]


def line_plane(p, d, q, n):
    return p + d * ((q - p).dot(n) / d.dot(n))


meta = {"units": "mm", "plate_top_z": None, "legs": []}
facets = {}

plate = gshape(parts("Top ASSY_Component5")[0])
joint_balls = [gshape(o) for o in parts("8mm Ball")]
facets["plate.stl"] = export(Part.makeCompound([plate] + joint_balls), "plate.stl")
meta["plate_top_z"] = max(f.CenterOfMass.z for f in plate.Faces
                          if type(f.Surface).__name__ == "Plane" and abs(f.normalAt(0, 0).z) > 0.999 and f.Area > 1e4)

facets["base.stl"] = export(gshape(parts("MidASSY")[0]), "base.stl", 0.25)
facets["camera.stl"] = export(Part.makeCompound([gshape(o) for o in parts("OV2710")]), "camera.stl")

ball_centres = [b.Solids[0].CenterOfMass for b in joint_balls]
cranks = [gshape(o) for o in parts("Leg ASSY_Component8")]
rods = [gshape(o) for o in parts("Leg ASSY_Component9")]
for k, crank in enumerate(cranks):
    hub = max(cylinders(crank, 10.75), key=lambda c: c[2])
    B = min(ball_centres, key=lambda c: (c - hub[0]).Length)
    A = line_plane(hub[0], hub[1], B, hub[1])
    P = min((line_plane(c[0], c[1], B, hub[1]) for c in cylinders(crank, 2.0)), key=lambda p: p.z)
    radial = App.Vector(A.x, A.y, 0).normalize()
    # rotation axis oriented so that a positive angle raises the crank pin
    axis = radial.cross(App.Vector(0, 0, 1)).normalize()
    rod = min(rods, key=lambda r: (r.BoundBox.Center - B).Length)
    facets["crank_%d.stl" % k] = export(crank, "crank_%d.stl" % k)
    facets["rod_%d.stl" % k] = export(rod, "rod_%d.stl" % k)
    meta["legs"].append({
        "azimuth_deg": math.degrees(math.atan2(B.y, B.x)),
        "crank_mesh": "crank_%d.stl" % k, "rod_mesh": "rod_%d.stl" % k,
        "pivot": [A.x, A.y, A.z], "axis": [axis.x, axis.y, axis.z],
        "pin": [P.x, P.y, P.z], "joint": [B.x, B.y, B.z],
        "crank_angle_deg": math.degrees(math.atan2((P - A).z, (P - A).dot(radial))),
    })

with open(os.path.join(OUT, "meshes.json"), "w") as f:
    json.dump(meta, f, indent=2)
print("exported", facets)
