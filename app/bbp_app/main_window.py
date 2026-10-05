"""Main window: project handling, the four tabs (Setup, Tracking, Control, Simulation),
compile-and-run, and the live simulation view."""
import json
from pathlib import Path

from PySide6.QtCore import QThread, QTimer, Qt, Signal
from PySide6.QtGui import QAction, QColor, QFont, QKeySequence, QTextCharFormat
from PySide6.QtWidgets import (QComboBox, QDoubleSpinBox, QFileDialog, QGridLayout, QGroupBox, QHBoxLayout,
                               QInputDialog, QLabel, QListWidget, QListWidgetItem, QMainWindow, QMessageBox,
                               QPlainTextEdit, QPushButton, QSplitter, QTabWidget, QToolBar, QVBoxLayout, QWidget)

from . import runner as RN
from . import settings_schema as schema
from . import toolchain
from .paths import CONFIG_FILE, PROJECTS_DIR
from .project import SOURCES, Project
from .widgets.api_reference import CONTROL, TRACKING, ApiReference
from .widgets.camera_view import CameraView
from .widgets.code_editor import CodeEditor
from .widgets.plots import LivePlots
from .widgets.setup_tab import SetupTab
from .widgets.view3d import PlatformView


class BuildThread(QThread):
    done = Signal(object)

    def __init__(self, project, parent=None):
        super().__init__(parent)
        self.project = project

    def run(self):
        try:
            self.done.emit(toolchain.build_project(self.project, SOURCES))
        except toolchain.ToolchainError as e:
            self.done.emit(e)
        except Exception as e:  # noqa: BLE001 - report anything to the user
            self.done.emit(toolchain.ToolchainError(f"Build failed: {e}"))


class EditorPage(QWidget):
    """Code editor + library reference + build messages for one source file."""
    jump = Signal(str, int)

    def __init__(self, filename, reference_html, parent=None):
        super().__init__(parent)
        self.filename = filename
        lay = QVBoxLayout(self)
        lay.setContentsMargins(4, 4, 4, 4)
        split = QSplitter(Qt.Horizontal)
        self.editor = CodeEditor()
        split.addWidget(self.editor)
        split.addWidget(ApiReference(reference_html))
        split.setStretchFactor(0, 3)
        split.setStretchFactor(1, 1)
        vsplit = QSplitter(Qt.Vertical)
        vsplit.addWidget(split)
        self.problems = QListWidget()
        self.problems.setMaximumHeight(140)
        self.problems.itemActivated.connect(self._activate)
        self.problems.itemClicked.connect(self._activate)
        vsplit.addWidget(self.problems)
        vsplit.setStretchFactor(0, 5)
        lay.addWidget(vsplit)

    def _activate(self, item):
        line = item.data(Qt.UserRole)
        if line:
            self.editor.goto_line(line)

    def show_diagnostics(self, diags):
        self.problems.clear()
        errors = {}
        for d in diags:
            item = QListWidgetItem(f"{d.severity}  line {d.line}:  {d.message}  ({d.code})")
            item.setData(Qt.UserRole, d.line)
            item.setForeground(QColor("#a01c1c" if d.severity == "error" else "#8a6100"))
            self.problems.addItem(item)
            if d.severity == "error":
                errors[d.line] = d.message
        self.editor.set_errors(errors)


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.resize(1500, 920)
        self.project = None
        self.build_thread = None
        self.pending_state = None
        self.last_meas = None
        self.ball_material = "steel"
        self.runner = RN.RunnerProcess(self)
        self.runner.stateReceived.connect(self._on_state)
        self.runner.frameReceived.connect(self._on_frame)
        self.runner.logReceived.connect(self._on_log)
        self.runner.textReceived.connect(self._on_text)
        self.runner.printed.connect(lambda t: self.console_write(t, "#2b2b2b"))
        self.runner.stopped.connect(self._on_stopped)

        self._build_toolbar()
        self.tabs = QTabWidget()
        self.setup = SetupTab()
        self.setup.changed.connect(self._settings_changed)
        self.pages = {"tracking.cpp": EditorPage("tracking.cpp", TRACKING),
                      "control.cpp": EditorPage("control.cpp", CONTROL)}
        self.tabs.addTab(self.setup, "Setup")
        self.tabs.addTab(self.pages["tracking.cpp"], "Tracking")
        self.tabs.addTab(self.pages["control.cpp"], "Control")
        self.tabs.addTab(self._build_sim_page(), "Simulation")
        self.setCentralWidget(self.tabs)
        self.status = QLabel()
        self.statusBar().addWidget(self.status, 1)

        self.timer = QTimer(self)
        self.timer.timeout.connect(self._refresh)
        self.timer.start(33)
        self.plot_timer = QTimer(self)
        self.plot_timer.timeout.connect(self._refresh_plots)
        self.plot_timer.start(100)
        self._open_initial_project()

    # ------------------------------------------------------------------ UI construction
    def _build_toolbar(self):
        tb = QToolBar("Main")
        tb.setMovable(False)
        tb.setToolButtonStyle(Qt.ToolButtonTextOnly)
        self.addToolBar(tb)

        def act(text, slot, shortcut=None, tip=None):
            a = QAction(text, self)
            a.triggered.connect(slot)
            if shortcut:
                a.setShortcut(QKeySequence(shortcut))
            if tip:
                a.setToolTip(tip)
            tb.addAction(a)
            return a

        act("New project", self.new_project, "Ctrl+N")
        act("Open project", self.open_project, "Ctrl+O")
        act("Save", self.save_all, "Ctrl+S")
        tb.addSeparator()
        self.run_action = act("▶  Run", self.run, "F5", "Compile your code and start the simulation (F5)")
        f = self.run_action.font()
        f.setBold(True)
        self.run_action.setFont(f)
        self.stop_action = act("■  Stop", self.stop_sim, "Shift+F5")
        self.pause_action = act("⏸  Pause", self.toggle_pause, "F6")
        tb.addSeparator()
        tb.addWidget(QLabel(" Speed "))
        self.speed = QComboBox()
        for v in (0.25, 0.5, 1.0, 2.0, 4.0):
            self.speed.addItem(f"{v:g}x", v)
        self.speed.setCurrentIndex(2)
        self.speed.currentIndexChanged.connect(self._speed_changed)
        tb.addWidget(self.speed)
        tb.addSeparator()
        self.project_label = QLabel()
        tb.addWidget(self.project_label)
        self._set_running(False)

    def _build_sim_page(self):
        page = QWidget()
        outer = QHBoxLayout(page)
        outer.setContentsMargins(4, 4, 4, 4)
        split = QSplitter(Qt.Horizontal)

        left = QSplitter(Qt.Vertical)
        self.view3d = PlatformView()
        left.addWidget(self.view3d)
        self.console = QPlainTextEdit()
        self.console.setReadOnly(True)
        self.console.setMaximumBlockCount(5000)
        self.console.setFont(QFont("Consolas", 9))
        left.addWidget(self.console)
        left.setStretchFactor(0, 4)
        left.setStretchFactor(1, 1)
        split.addWidget(left)

        right = QSplitter(Qt.Vertical)
        self.camera = CameraView()
        right.addWidget(self.camera)
        right.addWidget(self._build_controls())
        self.plots = LivePlots()
        right.addWidget(self.plots)
        right.setStretchFactor(0, 3)
        right.setStretchFactor(2, 4)
        split.addWidget(right)
        split.setStretchFactor(0, 3)
        split.setStretchFactor(1, 2)
        outer.addWidget(split)
        return page

    def _build_controls(self):
        box = QGroupBox()
        g = QGridLayout(box)
        g.setContentsMargins(6, 4, 6, 4)
        self.lbl_time = QLabel("t = 0.00 s")
        self.lbl_mode = QLabel("")
        self.lbl_cpu = QLabel("")
        self.lbl_cpu.setToolTip("Estimated run time of track() + control() on the FireBeetle "
                                "(PC time x the factor in Setup)")
        g.addWidget(self.lbl_time, 0, 0)
        g.addWidget(self.lbl_mode, 0, 1)
        g.addWidget(self.lbl_cpu, 0, 2, 1, 3)
        g.addWidget(QLabel("Live target [mm]"), 1, 0)
        self.tx = QDoubleSpinBox()
        self.ty = QDoubleSpinBox()
        for w in (self.tx, self.ty):
            w.setRange(-80, 80)
            w.setSingleStep(5)
            w.setDecimals(0)
        g.addWidget(self.tx, 1, 1)
        g.addWidget(self.ty, 1, 2)
        b = QPushButton("Set target")
        b.clicked.connect(lambda: self.runner.send(f"target {self.tx.value()} {self.ty.value()}"))
        g.addWidget(b, 1, 3)
        g.addWidget(QLabel("Nudge ball"), 2, 0)
        nudges = QHBoxLayout()
        for text, vx, vy in (("← -x", -150, 0), ("+x →", 150, 0), ("↓ -y", 0, -150), ("+y ↑", 0, 150)):
            nb = QPushButton(text)
            nb.clicked.connect(lambda _=False, a=vx, c=vy: self.runner.send(f"nudge {a} {c}"))
            nudges.addWidget(nb)
        g.addLayout(nudges, 2, 1, 1, 4)
        return box

    # ------------------------------------------------------------------ projects
    def _config(self):
        try:
            return json.loads(CONFIG_FILE.read_text(encoding="utf-8"))
        except Exception:
            return {}

    def _remember(self, path):
        cfg = self._config()
        cfg["last_project"] = str(path)
        try:
            CONFIG_FILE.write_text(json.dumps(cfg), encoding="utf-8")
        except OSError:
            pass

    def _open_initial_project(self):
        last = self._config().get("last_project")
        if last and Project.is_project(last):
            self.load_project(Project(Path(last)))
            return
        path = PROJECTS_DIR / "My first project"
        self.load_project(Project(path) if Project.is_project(path) else Project.create(path, "My first project"))

    def load_project(self, project):
        self.stop_sim()
        self.project = project
        for name, page in self.pages.items():
            page.editor.setPlainText(project.read_source(name))
            page.show_diagnostics([])
        self.setup.set_values(project.settings)
        self.project_label.setText(f"  Project: <b>{project.name}</b>  <span style='color:#6b6a65'>"
                                   f"({project.path})</span>")
        self.setWindowTitle(f"{project.name} - Ball Balancing Platform")
        self._remember(project.path)

    def new_project(self):
        name, ok = QInputDialog.getText(self, "New project", "Project name:")
        if not ok or not name.strip():
            return
        path = PROJECTS_DIR / name.strip()
        if Project.is_project(path):
            QMessageBox.warning(self, "New project", f"A project called '{name}' already exists.")
            return
        self.save_all()
        self.load_project(Project.create(path, name.strip()))

    def open_project(self):
        d = QFileDialog.getExistingDirectory(self, "Open project", str(PROJECTS_DIR))
        if not d:
            return
        if not Project.is_project(d):
            QMessageBox.warning(self, "Open project", "That folder is not a BBP project (no project.json).")
            return
        self.save_all()
        self.load_project(Project(Path(d)))

    def save_all(self):
        if not self.project:
            return
        for name, page in self.pages.items():
            self.project.write_source(name, page.editor.toPlainText())
        self.project.settings = self.setup.values()
        self.project.save_settings()
        self.status.setText("Saved.")

    def _settings_changed(self):
        if self.project:
            self.project.settings = self.setup.values()
            self.project.save_settings()

    # ------------------------------------------------------------------ build & run
    def run(self):
        if self.build_thread is not None:
            return
        self.save_all()
        self.stop_sim()
        self.console.clear()
        self.console_write("Compiling tracking.cpp and control.cpp ...", "#52514e")
        self.status.setText("Compiling...")
        self.run_action.setEnabled(False)
        self.build_thread = BuildThread(self.project, self)
        self.build_thread.done.connect(self._on_built)
        self.build_thread.start()

    def _on_built(self, result):
        self.build_thread.wait()
        self.build_thread = None
        self.run_action.setEnabled(True)
        if isinstance(result, toolchain.ToolchainError):
            self.console_write(str(result), "#a01c1c")
            self.status.setText("Build failed.")
            QMessageBox.critical(self, "Build failed", str(result))
            return
        for name, page in self.pages.items():
            page.show_diagnostics([d for d in result.diagnostics if d.file == name])
        if not result.ok:
            self.console_write(result.log, "#a01c1c")
            self.status.setText("Your code has errors - see the list under the editor.")
            first = next((d for d in result.diagnostics if d.severity == "error" and d.file in self.pages), None)
            if first:
                self.tabs.setCurrentWidget(self.pages[first.file])
                self.pages[first.file].editor.goto_line(first.line)
            return
        warnings = [d for d in result.diagnostics if d.severity == "warning"]
        self.console_write(f"Build OK in {result.seconds:.1f} s"
                           + (f" ({len(warnings)} warning(s))" if warnings else "") + ".", "#006300")
        self._start_runner(result.exe)

    def _start_runner(self, exe):
        values = self.setup.values()
        values["sim.speed"] = self.speed.currentData()
        settings_path = self.project.build_dir / "settings.txt"
        schema.write_settings_file(values, settings_path)
        self.ball_material = values["ball.material"]
        self.view3d.set_ball_material(self.ball_material)
        self.plots.clear()
        self.camera.clear()
        self.last_meas = None
        self.tx.setValue(values["target.x_mm"])
        self.ty.setValue(values["target.y_mm"])
        self.runner.start(exe, settings_path, self.project.build_dir)
        self.tabs.setCurrentIndex(3)
        self._set_running(True)
        self.status.setText("Running.")

    def stop_sim(self):
        if self.runner.is_running():
            self.runner.stop()
            self.console_write("Simulation stopped.", "#52514e")
        self._set_running(False)

    def toggle_pause(self):
        if not self.runner.is_running():
            return
        paused = self.pause_action.text().startswith("⏸")
        self.runner.send("pause" if paused else "resume")
        self.pause_action.setText("▶  Resume" if paused else "⏸  Pause")

    def _speed_changed(self):
        self.runner.send(f"speed {self.speed.currentData()}")

    def _set_running(self, running):
        self.stop_action.setEnabled(running)
        self.pause_action.setEnabled(running)
        self.pause_action.setText("⏸  Pause")
        self.run_action.setText("↻  Restart" if running else "▶  Run")

    def _on_stopped(self, code):
        self._set_running(False)
        if code != 0:
            self.console_write(f"The simulation process ended unexpectedly (exit code {code}). "
                               "This usually means your code crashed, e.g. reading outside the image.",
                               "#a01c1c")
            self.status.setText("Simulation crashed.")

    # ------------------------------------------------------------------ live data
    def _on_state(self, s):
        self.pending_state = s
        self.plots.add_state(s)
        if s[RN.S_MEAS_FOUND] > 0.5:
            m = (s[RN.S_MEAS], s[RN.S_MEAS + 1])
            if m != self.last_meas:
                self.last_meas = m
                self.plots.add_measurement(s[RN.S_T], *m)

    def _on_frame(self, f):
        self.camera.show_frame(f)

    def _on_log(self, name, t, v):
        self.plots.add_log(name, t, v)

    def _on_text(self, level, text):
        colour = ["#52514e", "#8a6100", "#a01c1c"][min(level, 2)]
        self.console_write(text, colour)
        if level >= 1:
            self.status.setText(text)

    def _refresh(self):
        s = self.pending_state
        if s is None:
            return
        self.pending_state = None
        self.view3d.update_state(s)
        mode = RN.MODES[int(s[RN.S_MODE])] if 0 <= int(s[RN.S_MODE]) < len(RN.MODES) else "?"
        rtf = s[RN.S_RTF]
        self.lbl_time.setText(f"t = {s[RN.S_T]:6.2f} s   ({rtf:.2f}x real time)" if rtf > 0 else
                              f"t = {s[RN.S_T]:6.2f} s   (paused)")
        self.lbl_mode.setText(f"ball: {mode}")
        self.lbl_cpu.setText(f"your code: {s[RN.S_HOST_MS]:.2f} ms on this PC, "
                             f"~{s[RN.S_CPU_MS]:.1f} ms on the FireBeetle")

    def _refresh_plots(self):
        if self.runner.is_running() and self.plots.series["x_true"].t:
            self.plots.refresh(self.plots.series["x_true"].t[-1])

    def console_write(self, text, colour="#0b0b0b"):
        fmt = QTextCharFormat()
        fmt.setForeground(QColor(colour))
        cur = self.console.textCursor()
        cur.movePosition(cur.MoveOperation.End)
        cur.insertText(text if text.endswith("\n") else text + "\n", fmt)
        self.console.setTextCursor(cur)
        self.console.ensureCursorVisible()

    def closeEvent(self, e):
        self.save_all()
        self.runner.stop()
        if self.build_thread:
            self.build_thread.wait(10000)
        super().closeEvent(e)
