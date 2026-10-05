"""3D view of the platform: the CAD meshes, moved to the simulated pose."""
import json
import math

import numpy as np
import vtkmodules.qt

vtkmodules.qt.PyQtImpl = "PySide6"
import vtkmodules.vtkInteractionStyle  # noqa: E402,F401
import vtkmodules.vtkRenderingOpenGL2  # noqa: E402,F401
from vtkmodules.qt.QVTKRenderWindowInteractor import QVTKRenderWindowInteractor  # noqa: E402
from vtkmodules.vtkCommonMath import vtkMatrix4x4  # noqa: E402
from vtkmodules.vtkFiltersCore import vtkPolyDataNormals, vtkQuadricDecimation  # noqa: E402
from vtkmodules.vtkFiltersSources import vtkCylinderSource, vtkSphereSource  # noqa: E402
from vtkmodules.vtkInteractionStyle import vtkInteractorStyleTrackballCamera  # noqa: E402
from vtkmodules.vtkIOGeometry import vtkSTLReader  # noqa: E402
from vtkmodules.vtkRenderingCore import vtkActor, vtkPolyDataMapper, vtkRenderer  # noqa: E402
from PySide6.QtWidgets import QVBoxLayout, QWidget  # noqa: E402

from .. import runner as RN  # noqa: E402
from ..paths import MESH_DIR  # noqa: E402

BALL_COLOURS = {
    "steel": (0.72, 0.73, 0.76),
    "glass": (0.55, 0.75, 0.70),
    "plastic_white": (0.95, 0.95, 0.93),
    "hollow_orange": (0.95, 0.45, 0.10),
}


def _hex(c):
    return tuple(int(c[i:i + 2], 16) / 255 for i in (1, 3, 5))


def translation(v):
    m = np.eye(4)
    m[:3, 3] = v
    return m


def rotation(axis, angle):
    a = np.asarray(axis, float)
    a = a / np.linalg.norm(a)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    m = np.eye(4)
    m[:3, :3] = np.eye(3) + math.sin(angle) * K + (1 - math.cos(angle)) * (K @ K)
    return m


def quat_matrix(w, x, y, z):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _vtk_matrix(m):
    vm = vtkMatrix4x4()
    for i in range(4):
        for j in range(4):
            vm.SetElement(i, j, float(m[i, j]))
    return vm


class PlatformView(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        self.vtk = QVTKRenderWindowInteractor(self)
        lay.addWidget(self.vtk)
        self.ren = vtkRenderer()
        self.ren.SetBackground(0.97, 0.97, 0.96)
        self.ren.SetBackground2(0.86, 0.87, 0.89)
        self.ren.GradientBackgroundOn()
        self.vtk.GetRenderWindow().AddRenderer(self.ren)
        self.vtk.GetRenderWindow().SetMultiSamples(4)
        self.vtk.SetInteractorStyle(vtkInteractorStyleTrackballCamera())

        self.meta = json.loads((MESH_DIR / "meshes.json").read_text())
        self.plate = self._mesh("plate.stl", "#9fd3e6", opacity=0.45)
        self._mesh("base.stl", "#5d5f63", decimate=0.6)
        self._mesh("camera.stl", "#2b2b2b")
        self.legs = []
        for leg in self.meta["legs"]:
            leg = dict(leg)
            leg["crank_actor"] = self._mesh(leg["crank_mesh"], "#d9822b")
            leg["rod_actor"] = self._mesh(leg["rod_mesh"], "#c9cbcf")
            self.legs.append(leg)

        sphere = vtkSphereSource()
        self.sphere = sphere
        sphere.SetRadius(10.0)
        sphere.SetThetaResolution(40)
        sphere.SetPhiResolution(30)
        self.ball = self._actor(sphere.GetOutputPort(), (0.72, 0.73, 0.76))
        self.ball.GetProperty().SetSpecular(0.8)
        self.ball.GetProperty().SetSpecularPower(60)

        marker = vtkCylinderSource()
        marker.SetRadius(4.0)
        marker.SetHeight(0.6)
        marker.SetResolution(32)
        self.target = self._actor(marker.GetOutputPort(), _hex("#eb6834"))
        self.target.SetOrientation(90, 0, 0)  # cylinder axis (y) -> z

        cam = self.ren.GetActiveCamera()
        cam.SetFocalPoint(0, 0, -30)
        cam.SetPosition(260, -330, 210)
        cam.SetViewUp(0, 0, 1)
        self.ren.ResetCameraClippingRange()
        self.plate_top = self.meta["plate_top_z"]
        self.vtk.Initialize()

    def _actor(self, port, colour, opacity=1.0):
        mapper = vtkPolyDataMapper()
        mapper.SetInputConnection(port)
        a = vtkActor()
        a.SetMapper(mapper)
        a.GetProperty().SetColor(*colour)
        a.GetProperty().SetOpacity(opacity)
        a.GetProperty().SetDiffuse(0.85)
        a.GetProperty().SetSpecular(0.15)
        self.ren.AddActor(a)
        return a

    def _mesh(self, name, colour, opacity=1.0, decimate=0.0):
        r = vtkSTLReader()
        r.SetFileName(str(MESH_DIR / name))
        port = r.GetOutputPort()
        if decimate > 0:
            d = vtkQuadricDecimation()
            d.SetInputConnection(port)
            d.SetTargetReduction(decimate)
            port = d.GetOutputPort()
        n = vtkPolyDataNormals()
        n.SetInputConnection(port)
        n.SetFeatureAngle(35)
        return self._actor(n.GetOutputPort(), _hex(colour), opacity)

    def set_ball(self, material, radius_mm):
        self.ball.GetProperty().SetColor(*BALL_COLOURS.get(material, BALL_COLOURS["steel"]))
        self.sphere.SetRadius(float(radius_mm))

    def update_state(self, s):
        # plate: CAD pose has R = I and the top face at z = plate_top
        R = quat_matrix(*s[RN.S_PLATE_Q:RN.S_PLATE_Q + 4])
        plate = np.eye(4)
        plate[:3, :3] = R
        plate[:3, 3] = s[RN.S_PLATE_P:RN.S_PLATE_P + 3]
        plate = plate @ translation([0, 0, -self.plate_top])
        self.plate.SetUserMatrix(_vtk_matrix(plate))

        for i in range(3):
            P = np.array(s[RN.S_PIN + 3 * i:RN.S_PIN + 3 * i + 3])
            B = np.array(s[RN.S_JOINT + 3 * i:RN.S_JOINT + 3 * i + 3])
            az = math.degrees(math.atan2(P[1], P[0]))
            leg = min(self.legs, key=lambda L: abs((L["azimuth_deg"] - az + 180) % 360 - 180))
            axis = np.array(leg["axis"])
            pivot = np.array(leg["pivot"])
            radial = np.array([pivot[0], pivot[1], 0.0])
            radial /= np.linalg.norm(radial)
            d_theta = s[RN.S_THETA + i] - math.radians(leg["crank_angle_deg"])
            crank = translation(pivot) @ rotation(axis, d_theta) @ translation(-pivot)
            leg["crank_actor"].SetUserMatrix(_vtk_matrix(crank))

            Pc, Bc = np.array(leg["pin"]), np.array(leg["joint"])
            dc, dn = Bc - Pc, B - P
            phi_c = math.atan2(dc[2], dc @ radial)
            phi_n = math.atan2(dn[2], dn @ radial)
            rod = translation(P) @ rotation(axis, phi_n - phi_c) @ translation(-Pc)
            leg["rod_actor"].SetUserMatrix(_vtk_matrix(rod))

        self.ball.SetPosition(*s[RN.S_BALL:RN.S_BALL + 3])
        # target marker on the plate surface
        t = plate @ np.array([s[RN.S_TARGET], s[RN.S_TARGET + 1], self.plate_top + 0.3, 1.0])
        tm = np.eye(4)
        tm[:3, :3] = R @ rotation([1, 0, 0], math.pi / 2)[:3, :3]
        tm[:3, 3] = t[:3]
        self.target.SetOrientation(0, 0, 0)
        self.target.SetUserMatrix(_vtk_matrix(tm))
        self.vtk.GetRenderWindow().Render()
