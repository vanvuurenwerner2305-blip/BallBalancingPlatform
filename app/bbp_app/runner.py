"""Talks to a running simulation (the project's runner executable): starts it, sends
commands and decodes its message stream. See framework/src/runner_main.cpp for the format."""
import struct

import numpy as np
from PySide6.QtCore import QObject, QProcess, Signal

MSG_STATE, MSG_FRAME, MSG_LOG, MSG_TEXT = 1, 2, 3, 4
STATE_SIZE = 48

# Indices into the STATE array (keep in sync with runner_main.cpp)
S_T, S_RTF = 0, 1
S_PLATE_P, S_PLATE_Q, S_THETA, S_PIN, S_JOINT, S_BALL = 2, 5, 9, 12, 21, 30
S_BALL_PLATE, S_MEAS_FOUND, S_MEAS, S_TARGET, S_CMD, S_SLOPE = 33, 35, 36, 38, 40, 42
S_MODE, S_CPU_MS, S_PAUSED, S_HOST_MS = 44, 45, 46, 47

MODES = ["rolling", "stuck", "slipping", "in the air", "fell off"]

FRAME_HDR = struct.Struct("<IIIIdBff")


class Frame:
    __slots__ = ("index", "width", "height", "fmt", "t", "found", "x", "y", "data")


class RunnerProcess(QObject):
    stateReceived = Signal(object)        # numpy array of STATE_SIZE
    frameReceived = Signal(object)        # Frame
    logReceived = Signal(str, float, float)  # name, t, value
    textReceived = Signal(int, str)       # level, text
    printed = Signal(str)                 # student's printf output
    stopped = Signal(int)                 # exit code

    def __init__(self, parent=None):
        super().__init__(parent)
        self.proc = None
        self.buf = bytearray()

    def is_running(self):
        return self.proc is not None and self.proc.state() != QProcess.NotRunning

    def start(self, exe, settings_path, workdir):
        self.stop()
        self.buf = bytearray()
        self.proc = QProcess(self)
        self.proc.setWorkingDirectory(str(workdir))
        self.proc.readyReadStandardOutput.connect(self._on_stdout)
        self.proc.readyReadStandardError.connect(self._on_stderr)
        self.proc.finished.connect(self._on_finished)
        self.proc.start(str(exe), [str(settings_path)])

    def send(self, command: str):
        if self.is_running():
            self.proc.write((command + "\n").encode())

    def stop(self):
        if self.proc is None:
            return
        p, self.proc = self.proc, None
        try:
            p.finished.disconnect(self._on_finished)
        except (RuntimeError, TypeError):
            pass
        if p.state() != QProcess.NotRunning:
            p.write(b"quit\n")
            if not p.waitForFinished(500):
                p.kill()
                p.waitForFinished(1000)
        p.deleteLater()

    def _on_finished(self, code, _status):
        self.stopped.emit(int(code))

    def _on_stderr(self):
        if self.proc:
            text = bytes(self.proc.readAllStandardError()).decode("utf-8", "replace")
            if text:
                self.printed.emit(text)

    def _on_stdout(self):
        if not self.proc:
            return
        self.buf += bytes(self.proc.readAllStandardOutput())
        buf = self.buf
        pos = 0
        latest_state = None
        latest_frame = None
        while len(buf) - pos >= 8:
            typ, ln = struct.unpack_from("<II", buf, pos)
            if len(buf) - pos - 8 < ln:
                break
            payload = bytes(buf[pos + 8:pos + 8 + ln])
            pos += 8 + ln
            if typ == MSG_STATE:
                latest_state = np.frombuffer(payload, dtype="<f8", count=STATE_SIZE).copy()
            elif typ == MSG_FRAME:
                f = Frame()
                f.index, f.width, f.height, f.fmt, f.t, found, f.x, f.y = FRAME_HDR.unpack_from(payload, 0)
                f.found = bool(found)
                f.data = payload[FRAME_HDR.size:]
                latest_frame = f
            elif typ == MSG_LOG:
                t, v = struct.unpack_from("<dd", payload, 0)
                self.logReceived.emit(payload[16:].decode("utf-8", "replace"), t, v)
            elif typ == MSG_TEXT:
                (level,) = struct.unpack_from("<I", payload, 0)
                self.textReceived.emit(int(level), payload[4:].decode("utf-8", "replace"))
        del buf[:pos]
        # only the newest state/frame matter for display
        if latest_state is not None:
            self.stateReceived.emit(latest_state)
        if latest_frame is not None:
            self.frameReceived.emit(latest_frame)
