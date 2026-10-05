import sys

from PySide6.QtWidgets import QApplication


def _light_palette(app):
    """A fixed light theme, so the app looks the same whatever the Windows colour mode is."""
    from PySide6.QtGui import QColor, QPalette

    p = QPalette()
    roles = {
        QPalette.Window: "#f4f4f1", QPalette.WindowText: "#0b0b0b", QPalette.Base: "#ffffff",
        QPalette.AlternateBase: "#f1f0ec", QPalette.Text: "#0b0b0b", QPalette.Button: "#ecebe6",
        QPalette.ButtonText: "#0b0b0b", QPalette.Highlight: "#2a78d6", QPalette.HighlightedText: "#ffffff",
        QPalette.ToolTipBase: "#ffffe8", QPalette.ToolTipText: "#0b0b0b", QPalette.PlaceholderText: "#898781",
        QPalette.Link: "#2a78d6",
    }
    for role, colour in roles.items():
        p.setColor(role, QColor(colour))
    p.setColor(QPalette.Disabled, QPalette.Text, QColor("#a5a39b"))
    p.setColor(QPalette.Disabled, QPalette.ButtonText, QColor("#a5a39b"))
    p.setColor(QPalette.Disabled, QPalette.WindowText, QColor("#a5a39b"))
    app.setPalette(p)


def main():
    if sys.platform == "win32":
        # Own taskbar identity, so Windows shows our icon instead of python's.
        import ctypes

        ctypes.windll.shell32.SetCurrentProcessExplicitAppUserModelID("BBP.BallBalancingPlatform")
    app = QApplication(sys.argv)
    app.setApplicationName("Ball Balancing Platform")
    from PySide6.QtGui import QIcon

    from .paths import REPO_ROOT

    app.setWindowIcon(QIcon(str(REPO_ROOT / "app" / "assets" / "bbp.ico")))
    app.setStyle("Fusion")
    _light_palette(app)
    from .main_window import MainWindow  # after QApplication exists (VTK/Qt setup)

    w = MainWindow()
    w.show()
    return app.exec()
