"""Documentation: the list of guides, opened in the system PDF viewer."""
from PySide6.QtCore import Qt, QUrl
from PySide6.QtGui import QDesktopServices
from PySide6.QtWidgets import (QDialog, QHBoxLayout, QLabel, QListWidget, QListWidgetItem, QMessageBox,
                               QPushButton, QVBoxLayout)

from ..paths import REPO_ROOT

DOCS_DIR = REPO_ROOT / "app" / "assets" / "docs"

DOCUMENTS = [
    ("Vision library", "bbp_vision_library.pdf",
     "For tracking.cpp: the camera image, what the ball looks like, tracking techniques and the "
     "FireBeetle time budget."),
    ("Control library", "bbp_control_library.pdf",
     "For control.cpp: the Ball, Target and Platform, the PID helper, timing and delays, and how to "
     "design a controller for the platform."),
    ("Physics model", "bbp_physics_model.pdf",
     "How the simulator works: kinematics, servo and ball dynamics (including rolling inertia), "
     "camera image formation, and verification."),
]


class DocsDialog(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Documentation")
        self.resize(560, 330)
        lay = QVBoxLayout(self)
        lay.addWidget(QLabel("<b>Documentation</b> &nbsp; <span style='color:#6b6a65'>Double-click to open "
                             "(PDF).</span>"))
        self.list = QListWidget()
        self.list.setWordWrap(True)
        self.list.setSpacing(4)
        for title, filename, description in DOCUMENTS:
            item = QListWidgetItem(f"{title}\n{description}")
            item.setData(Qt.UserRole, filename)
            f = item.font()
            item.setFont(f)
            self.list.addItem(item)
        self.list.setCurrentRow(0)
        self.list.itemActivated.connect(self._open)
        lay.addWidget(self.list)
        buttons = QHBoxLayout()
        buttons.addStretch(1)
        open_btn = QPushButton("Open")
        open_btn.setDefault(True)
        open_btn.clicked.connect(lambda: self._open(self.list.currentItem()))
        close = QPushButton("Close")
        close.clicked.connect(self.close)
        buttons.addWidget(open_btn)
        buttons.addWidget(close)
        lay.addLayout(buttons)

    def _open(self, item):
        if item is None:
            return
        path = DOCS_DIR / item.data(Qt.UserRole)
        if not path.exists():
            QMessageBox.warning(self, "Documentation", f"{path.name} was not found.\n"
                                "Build the documents with tools/build_docs.sh.")
            return
        QDesktopServices.openUrl(QUrl.fromLocalFile(str(path)))
