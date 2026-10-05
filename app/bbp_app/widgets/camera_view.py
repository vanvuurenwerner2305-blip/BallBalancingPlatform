"""Live view of the frames handed to track(), with the student's detection drawn on top."""
import numpy as np
from PySide6.QtCore import QPointF, QRectF, Qt
from PySide6.QtGui import QColor, QImage, QPainter, QPen
from PySide6.QtWidgets import QWidget


def frame_to_rgb(f):
    if f.fmt == 1:  # RGB565, big-endian
        v = np.frombuffer(f.data, dtype=">u2").reshape(f.height, f.width).astype(np.uint16)
        r = ((v >> 11) & 31).astype(np.uint32) * 255 // 31
        g = ((v >> 5) & 63).astype(np.uint32) * 255 // 63
        b = (v & 31).astype(np.uint32) * 255 // 31
        return np.dstack([r, g, b]).astype(np.uint8)
    if f.fmt == 0:
        g = np.frombuffer(f.data, dtype=np.uint8).reshape(f.height, f.width)
        return np.dstack([g, g, g])
    return np.frombuffer(f.data, dtype=np.uint8).reshape(f.height, f.width, 3)


class CameraView(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.image = None
        self.frame = None
        self.setMinimumSize(320, 240)

    def show_frame(self, f):
        rgb = np.ascontiguousarray(frame_to_rgb(f))
        self.image = QImage(rgb.data, f.width, f.height, 3 * f.width, QImage.Format_RGB888).copy()
        self.frame = f
        self.update()

    def clear(self):
        self.image = None
        self.frame = None
        self.update()

    def paintEvent(self, _):
        p = QPainter(self)
        p.fillRect(self.rect(), QColor("#1a1a19"))
        if self.image is None:
            p.setPen(QColor("#898781"))
            p.drawText(self.rect(), Qt.AlignCenter, "Camera\n(press Run)")
            return
        w, h = self.image.width(), self.image.height()
        scale = min(self.width() / w, self.height() / h)
        dw, dh = w * scale, h * scale
        x0, y0 = (self.width() - dw) / 2, (self.height() - dh) / 2
        p.setRenderHint(QPainter.SmoothPixmapTransform, False)
        p.drawImage(QRectF(x0, y0, dw, dh), self.image)
        f = self.frame
        if f.found:
            c = QPointF(x0 + f.x * scale, y0 + f.y * scale)
            p.setRenderHint(QPainter.Antialiasing, True)
            p.setPen(QPen(QColor("#1baf7a"), 2))
            r = 6 * scale
            p.drawLine(c + QPointF(-r, 0), c + QPointF(r, 0))
            p.drawLine(c + QPointF(0, -r), c + QPointF(0, r))
            p.drawEllipse(c, 2.2 * r, 2.2 * r)
        p.setPen(QColor("#ffffff"))
        status = f"frame {f.index}   t = {f.t:6.2f} s   " + (
            f"ball at ({f.x:.1f}, {f.y:.1f}) px" if f.found else "ball not found")
        p.fillRect(QRectF(x0, y0 + dh - 20, dw, 20), QColor(0, 0, 0, 140))
        p.drawText(QRectF(x0 + 6, y0 + dh - 20, dw - 12, 20), Qt.AlignVCenter | Qt.AlignLeft, status)
