"""Settings forms generated from settings_schema: the Setup tab (experiment) and the
Simulated hardware dialog (the simulator's physical model)."""
from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import (QCheckBox, QComboBox, QDialog, QDoubleSpinBox, QFormLayout, QGridLayout, QGroupBox,
                               QHBoxLayout, QLabel, QPushButton, QScrollArea, QSpinBox, QVBoxLayout, QWidget)

from .. import settings_schema as schema


class SettingsForm(QWidget):
    changed = Signal()

    def __init__(self, groups, intro, parent=None):
        super().__init__(parent)
        self.groups = groups
        self.editors = {}
        self._applying_preset = False
        outer = QVBoxLayout(self)
        head = QHBoxLayout()
        head.addWidget(QLabel(intro))
        head.addStretch(1)
        reset = QPushButton("Restore defaults")
        reset.clicked.connect(self.restore_defaults)
        head.addWidget(reset)
        outer.addLayout(head)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        body = QWidget()
        grid = QGridLayout(body)
        grid.setHorizontalSpacing(16)
        for gi, group in enumerate(groups):
            box = QGroupBox(group)
            form = QFormLayout(box)
            for s in (s for s in schema.SETTINGS if s.group == group):
                w = self._make_editor(s)
                self.editors[s.key] = w
                label = QLabel(s.label + (f" [{s.unit}]" if s.unit else ""))
                if s.help:
                    label.setToolTip(s.help)
                    w.setToolTip(s.help)
                form.addRow(label, w)
            grid.addWidget(box, gi // 2, gi % 2, Qt.AlignTop)
        grid.setRowStretch((len(groups) + 1) // 2, 1)
        scroll.setWidget(body)
        outer.addWidget(scroll)

    def _make_editor(self, s):
        if s.kind == "bool":
            w = QCheckBox()
            w.toggled.connect(lambda _, k=s.key: self._edited(k))
        elif s.kind == "choice":
            w = QComboBox()
            for value, label in s.choices:
                w.addItem(label, value)
            w.currentIndexChanged.connect(lambda _, k=s.key: self._edited(k))
        elif s.kind == "int":
            w = QSpinBox()
            w.setRange(int(s.minimum), int(s.maximum))
            w.valueChanged.connect(lambda _, k=s.key: self._edited(k))
        else:
            w = QDoubleSpinBox()
            w.setRange(s.minimum, s.maximum)
            w.setSingleStep(s.step)
            w.setDecimals(s.decimals)
            w.valueChanged.connect(lambda _, k=s.key: self._edited(k))
        return w

    def _edited(self, key):
        if self._applying_preset:
            return
        s = schema.BY_KEY[key]
        if s.presets:
            values = s.presets.get(self._value(key))
            if values:
                self._applying_preset = True
                self.set_values(values)
                self._applying_preset = False
        else:
            # editing a value a preset controls turns that preset into "custom"
            for p in schema.SETTINGS:
                if p.presets and p.key in self.editors and any(key in v for v in p.presets.values()):
                    current = p.presets.get(self._value(p.key), {})
                    if key in current and current[key] != self._value(key):
                        self._applying_preset = True
                        self.set_values({p.key: "custom"})
                        self._applying_preset = False
        self.changed.emit()

    def _value(self, key):
        w = self.editors[key]
        if isinstance(w, QCheckBox):
            return w.isChecked()
        if isinstance(w, QComboBox):
            return w.currentData()
        return w.value()

    def set_values(self, values):
        for key, v in values.items():
            w = self.editors.get(key)
            if w is None:
                continue
            w.blockSignals(True)
            if isinstance(w, QCheckBox):
                w.setChecked(bool(v))
            elif isinstance(w, QComboBox):
                w.setCurrentIndex(max(0, w.findData(v)))
            else:
                w.setValue(v)
            w.blockSignals(False)

    def values(self):
        return {key: self._value(key) for key in self.editors}

    def restore_defaults(self):
        self.set_values({s.key: s.default for s in schema.SETTINGS if s.group in self.groups})
        self.changed.emit()


class SetupTab(SettingsForm):
    def __init__(self, parent=None):
        super().__init__(schema.SETUP_GROUPS,
                         "<b>Setup</b> &nbsp; <span style='color:#6b6a65'>The experiment: applies in the simulator "
                         "and on the real platform. Changes apply the next time you press Run.</span>", parent)


class ModelDialog(QDialog):
    """Simulated hardware: the physical parameters of the simulator."""
    applyRequested = Signal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Simulated hardware")
        self.resize(900, 640)
        lay = QVBoxLayout(self)
        self.form = SettingsForm(schema.MODEL_GROUPS,
                                 "<b>Simulated hardware</b> &nbsp; <span style='color:#6b6a65'>The simulator's "
                                 "physical model. These only exist in the simulator; on the real platform the "
                                 "hardware decides. See Documentation &gt; Physics model.</span>")
        lay.addWidget(self.form)
        buttons = QHBoxLayout()
        buttons.addStretch(1)
        apply = QPushButton("Apply and restart simulation")
        apply.setDefault(True)
        apply.clicked.connect(self.applyRequested.emit)
        close = QPushButton("Close")
        close.clicked.connect(self.close)
        buttons.addWidget(apply)
        buttons.addWidget(close)
        lay.addLayout(buttons)
