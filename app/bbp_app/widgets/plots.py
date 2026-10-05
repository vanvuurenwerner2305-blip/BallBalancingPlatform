"""Live plots: ball position against target, plate angles, and the student's log() values."""
from collections import deque

import pyqtgraph as pg
from PySide6.QtWidgets import QVBoxLayout, QWidget

from .. import runner as RN

# reference categorical palette (fixed order); line style carries identity too
BLUE, ORANGE, AQUA, YELLOW, MAGENTA, GREEN = "#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#008300"
LOG_COLOURS = [AQUA, YELLOW, MAGENTA, GREEN, BLUE, ORANGE]
WINDOW = 10.0  # seconds shown


class _Series:
    def __init__(self, plot, name, colour, style=None, width=2, symbol=None):
        pen = pg.mkPen(colour, width=width, style=style) if style else pg.mkPen(colour, width=width)
        kw = dict(symbol=symbol, symbolSize=4, symbolBrush=colour, symbolPen=None) if symbol else {}
        self.curve = plot.plot([], [], pen=None if symbol else pen, name=name, **kw)
        self.t = deque(maxlen=3000)
        self.v = deque(maxlen=3000)

    def add(self, t, v):
        self.t.append(t)
        self.v.append(v)

    def refresh(self):
        self.curve.setData(list(self.t), list(self.v))

    def clear(self):
        self.t.clear()
        self.v.clear()
        self.curve.setData([], [])


class LivePlots(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        pg.setConfigOptions(antialias=True, background="#fcfcfb", foreground="#52514e")
        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        self.win = pg.GraphicsLayoutWidget()
        lay.addWidget(self.win)
        from PySide6.QtCore import Qt
        dash = Qt.DashLine

        def mkplot(row, title, unit):
            p = self.win.addPlot(row=row, col=0)
            p.showGrid(x=True, y=True, alpha=0.25)
            p.setLabel("left", f"{title} [{unit}]" if unit else title)
            legend = p.addLegend(offset=(-5, 2), labelTextSize="7pt", colCount=3)
            legend.setBrush(pg.mkBrush(252, 252, 251, 200))
            p.setMinimumHeight(110)
            return p

        self.px = mkplot(0, "ball x", "mm")
        self.py = mkplot(1, "ball y", "mm")
        self.pa = mkplot(2, "plate angle", "deg")
        self.pl = mkplot(3, "log()", "")
        for p in (self.py, self.pa, self.pl):
            p.setXLink(self.px)
        self.pl.setLabel("bottom", "time", "s")
        self.series = {
            "x_true": _Series(self.px, "true", BLUE),
            "x_meas": _Series(self.px, "measured", ORANGE, symbol="o"),
            "x_target": _Series(self.px, "target", "#0b0b0b", dash, 1),
            "y_true": _Series(self.py, "true", BLUE),
            "y_meas": _Series(self.py, "measured", ORANGE, symbol="o"),
            "y_target": _Series(self.py, "target", "#0b0b0b", dash, 1),
            "ax": _Series(self.pa, "angleX", BLUE),
            "ay": _Series(self.pa, "angleY", ORANGE, dash),
        }
        self.logs = {}
        self.last_meas_t = -1

    def clear(self):
        for s in list(self.series.values()) + list(self.logs.values()):
            s.clear()
        for s in self.logs.values():
            self.pl.removeItem(s.curve)
        self.logs = {}
        self.pl.clear()
        legend = self.pl.addLegend(offset=(-5, 2), labelTextSize="7pt", colCount=3)
        legend.setBrush(pg.mkBrush(252, 252, 251, 200))

    def add_state(self, s):
        t = s[RN.S_T]
        S = self.series
        S["x_true"].add(t, s[RN.S_BALL_PLATE])
        S["y_true"].add(t, s[RN.S_BALL_PLATE + 1])
        S["x_target"].add(t, s[RN.S_TARGET])
        S["y_target"].add(t, s[RN.S_TARGET + 1])
        S["ax"].add(t, s[RN.S_CMD])
        S["ay"].add(t, s[RN.S_CMD + 1])

    def add_measurement(self, t, x, y):
        self.series["x_meas"].add(t, x)
        self.series["y_meas"].add(t, y)

    def add_log(self, name, t, v):
        if name not in self.logs:
            if len(self.logs) >= len(LOG_COLOURS):
                return  # more series than distinguishable colours: ignore extras
            self.logs[name] = _Series(self.pl, name, LOG_COLOURS[len(self.logs)])
        self.logs[name].add(t, v)

    def refresh(self, t_now):
        for s in list(self.series.values()) + list(self.logs.values()):
            s.refresh()
        self.px.setXRange(max(0.0, t_now - WINDOW), max(WINDOW, t_now), padding=0)
