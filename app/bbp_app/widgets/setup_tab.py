"""The Setup tab: a form generated from settings_schema.SETTINGS."""
from PySide6.QtCore import Signal
from PySide6.QtWidgets import (QCheckBox, QComboBox, QDoubleSpinBox, QFormLayout, QGridLayout, QGroupBox,
                               QHBoxLayout, QLabel, QPushButton, QScrollArea, QSpinBox, QVBoxLayout, QWidget)

from .. import settings_schema as schema


class SetupTab(QWidget):
    changed = Signal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self.editors = {}
        outer = QVBoxLayout(self)
        head = QHBoxLayout()
        title = QLabel("<b>Setup</b> &nbsp; <span style='color:#6b6a65'>Changes apply the next time you press "
                       "Run (simulation speed applies immediately).</span>")
        head.addWidget(title)
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
        for gi, group in enumerate(schema.GROUPS):
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
            grid.addWidget(box, gi // 2, gi % 2)
        grid.setRowStretch(len(schema.GROUPS) // 2 + 1, 1)
        scroll.setWidget(body)
        outer.addWidget(scroll)

    def _make_editor(self, s):
        if s.kind == "bool":
            w = QCheckBox()
            w.toggled.connect(lambda _: self.changed.emit())
        elif s.kind == "choice":
            w = QComboBox()
            for value, label in s.choices:
                w.addItem(label, value)
            w.currentIndexChanged.connect(lambda _: self.changed.emit())
        elif s.kind == "int":
            w = QSpinBox()
            w.setRange(int(s.minimum), int(s.maximum))
            w.valueChanged.connect(lambda _: self.changed.emit())
        else:
            w = QDoubleSpinBox()
            w.setRange(s.minimum, s.maximum)
            w.setSingleStep(s.step)
            w.setDecimals(s.decimals)
            w.valueChanged.connect(lambda _: self.changed.emit())
        return w

    def set_values(self, values):
        for key, w in self.editors.items():
            v = values.get(key, schema.BY_KEY[key].default)
            w.blockSignals(True)
            if isinstance(w, QCheckBox):
                w.setChecked(bool(v))
            elif isinstance(w, QComboBox):
                i = w.findData(v)
                w.setCurrentIndex(max(0, i))
            else:
                w.setValue(v)
            w.blockSignals(False)

    def values(self):
        out = {}
        for key, w in self.editors.items():
            if isinstance(w, QCheckBox):
                out[key] = w.isChecked()
            elif isinstance(w, QComboBox):
                out[key] = w.currentData()
            else:
                out[key] = w.value()
        return out

    def restore_defaults(self):
        self.set_values(schema.defaults())
        self.changed.emit()
