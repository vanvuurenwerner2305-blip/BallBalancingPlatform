"""The 'library' panel next to the editors: what students can read and what they can set."""
from PySide6.QtWidgets import QTextBrowser

STYLE = """
<style>
 body { font-family: 'Segoe UI'; font-size: 9pt; color: #1b1b1a; }
 h3 { margin: 10px 0 4px 0; color: #0b0b0b; }
 code, .c { font-family: Consolas; color: #5b2b8a; }
 td { padding: 2px 6px 2px 0; vertical-align: top; }
 .k { font-family: Consolas; color: #5b2b8a; white-space: nowrap; }
 .muted { color: #6b6a65; }
</style>
"""


def _table(rows):
    return "<table>" + "".join(f"<tr><td class='k'>{a}</td><td>{b}</td></tr>" for a, b in rows) + "</table>"


TRACKING = STYLE + """
<h3>You write</h3>
<code>void track(const Image&amp; image, Detection&amp; ball)</code>
<p class='muted'>Called for every camera frame. Report the ball centre in <b>pixels</b>.</p>
<h3>Image (read)</h3>
""" + _table([
    ("image.width()", "image width in pixels (320)"),
    ("image.height()", "image height in pixels (240)"),
    ("image.gray(x, y)", "brightness 0..255"),
    ("image.red(x, y)", "red 0..255"),
    ("image.green(x, y)", "green 0..255"),
    ("image.blue(x, y)", "blue 0..255"),
]) + """
<p class='muted'>Pixel (0,0) is the top-left corner; x grows right, y grows down.
The camera looks <b>up</b> through the plate: the ball is a dark disc on the bright ceiling.</p>
<h3>Detection (set)</h3>
""" + _table([
    ("ball.found", "true if you found the ball"),
    ("ball.x", "pixel column of the ball centre"),
    ("ball.y", "pixel row of the ball centre"),
]) + """
<h3>Helpers</h3>
""" + _table([
    ("log(\"name\", value)", "plot a value live in the Simulation tab"),
    ("printf(...)", "print to the output console"),
])

CONTROL = STYLE + """
<h3>You write</h3>
<code>void control(const Ball&amp; ball, const Target&amp; target, Platform&amp; platform)</code>
<p class='muted'>Called after every track(). The library has turned your pixel position into
millimetres on the plate (origin at the centre).</p>
<h3>Ball (read)</h3>
""" + _table([
    ("ball.found", "false if track() lost the ball"),
    ("ball.x, ball.y", "position [mm]"),
    ("ball.vx, ball.vy", "velocity [mm/s], filtered (Setup &gt; Control)"),
    ("ball.dt", "time since the previous call [s]"),
    ("ball.time", "current time [s]"),
    ("ball.history(k)", "k-th previous sample: .x .y .t (k = 0 newest)"),
    ("ball.historySize()", "number of stored samples (max 64)"),
]) + """
<h3>Target (read)</h3>
""" + _table([("target.x, target.y", "where the ball should go [mm]")]) + """
<h3>Platform (set)</h3>
""" + _table([
    ("platform.angleX", "plate tilt along x [deg]. Positive lifts the +x edge, so the ball rolls to -x"),
    ("platform.angleY", "same for y"),
]) + """
<h3>PID controller</h3>
""" + _table([
    ("PID pid(kp, ki, kd);", "create one (outside the function, so it keeps its state)"),
    ("pid.update(error, dt)", "returns kp*e + ki*&int;e + kd*de/dt"),
    ("pid.setDerivativeFilter(s)", "low-pass the derivative, time constant s seconds"),
    ("pid.setIntegralLimit(l)", "anti-windup"),
    ("pid.setOutputLimit(l)", "clamp the output"),
    ("pid.reset()", "clear integral and history"),
    ("pid.lastP / lastI / lastD", "the three terms of the last update"),
]) + """
<h3>Helpers</h3>
""" + _table([
    ("log(\"name\", value)", "plot a value live in the Simulation tab"),
    ("clamp(v, lo, hi)", "limit a value"),
    ("printf(...)", "print to the output console"),
])


class ApiReference(QTextBrowser):
    def __init__(self, html, parent=None):
        super().__init__(parent)
        self.setHtml(html)
        self.setOpenExternalLinks(False)
        self.setMinimumWidth(280)
