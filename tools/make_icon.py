"""Draw the app icon (a ball on a tilted plate held up by three legs) and save it as
app/assets/bbp.ico (multi-size) and app/assets/bbp.png. Run with the app's environment:

    <bbp env>/python.exe tools/make_icon.py
"""
import sys
from pathlib import Path

from PySide6.QtCore import QPointF, QRectF, Qt
from PySide6.QtGui import QBrush, QColor, QImage, QLinearGradient, QPainter, QPainterPath, QPen, QRadialGradient
from PySide6.QtWidgets import QApplication

OUT = Path(__file__).resolve().parents[1] / "app" / "assets"


def draw(size: int) -> QImage:
    img = QImage(size, size, QImage.Format_ARGB32)
    img.fill(Qt.transparent)
    p = QPainter(img)
    p.setRenderHint(QPainter.Antialiasing)
    p.scale(size / 256.0, size / 256.0)

    # rounded-square tile
    bg = QLinearGradient(0, 0, 0, 256)
    bg.setColorAt(0, QColor("#2a78d6"))
    bg.setColorAt(1, QColor("#104281"))
    tile = QPainterPath()
    tile.addRoundedRect(QRectF(8, 8, 240, 240), 52, 52)
    p.fillPath(tile, QBrush(bg))

    # three legs
    p.setPen(QPen(QColor("#f0efec"), 13, Qt.SolidLine, Qt.RoundCap))
    for x0, y0, x1, y1 in ((70, 214, 64, 150), (186, 214, 194, 138), (128, 226, 128, 168)):
        p.drawLine(QPointF(x0, y0), QPointF(x1, y1))

    # tilted plate (ellipse seen from the side, rotated)
    p.save()
    p.translate(128, 146)
    p.rotate(-9)
    p.setPen(QPen(QColor("#0d366b"), 5))
    plate = QLinearGradient(0, -26, 0, 26)
    plate.setColorAt(0, QColor("#e8f3fd"))
    plate.setColorAt(1, QColor("#9ec5f4"))
    p.setBrush(QBrush(plate))
    p.drawEllipse(QRectF(-98, -26, 196, 52))
    p.restore()

    # ball, rolling off-centre towards the low side
    ball = QRadialGradient(QPointF(140, 88), 46, QPointF(126, 74))
    ball.setColorAt(0, QColor("#ffd2a8"))
    ball.setColorAt(0.45, QColor("#eb6834"))
    ball.setColorAt(1, QColor("#9c3a12"))
    p.setPen(Qt.NoPen)
    p.setBrush(QBrush(ball))
    p.drawEllipse(QPointF(146, 100), 34, 34)
    p.end()
    return img


def main():
    app = QApplication(sys.argv)  # noqa: F841 - needed for QImage text/plugins
    OUT.mkdir(parents=True, exist_ok=True)
    draw(256).save(str(OUT / "bbp.png"))
    # Qt's ICO writer stores one image; build a multi-size .ico by hand (PNG-compressed entries).
    import io
    import struct

    from PySide6.QtCore import QBuffer, QByteArray, QIODevice

    entries = []
    for s in (16, 24, 32, 48, 64, 128, 256):
        ba = QByteArray()
        buf = QBuffer(ba)
        buf.open(QIODevice.WriteOnly)
        draw(s).save(buf, "PNG")
        entries.append((s, bytes(ba)))
    out = io.BytesIO()
    out.write(struct.pack("<HHH", 0, 1, len(entries)))
    offset = 6 + 16 * len(entries)
    for s, data in entries:
        out.write(struct.pack("<BBBBHHII", s % 256, s % 256, 0, 0, 1, 32, len(data), offset))
        offset += len(data)
    for _, data in entries:
        out.write(data)
    (OUT / "bbp.ico").write_bytes(out.getvalue())
    print("wrote", OUT / "bbp.ico")


if __name__ == "__main__":
    main()
