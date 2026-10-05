"""C++ code editor with line numbers, syntax highlighting and error markers."""
import re

from PySide6.QtCore import QRect, QSize, Qt
from PySide6.QtGui import (QColor, QFont, QFontDatabase, QPainter, QSyntaxHighlighter, QTextCharFormat,
                           QTextCursor, QTextFormat)
from PySide6.QtWidgets import QPlainTextEdit, QTextEdit, QWidget


def _fmt(color, bold=False, italic=False):
    f = QTextCharFormat()
    f.setForeground(QColor(color))
    if bold:
        f.setFontWeight(QFont.Bold)
    f.setFontItalic(italic)
    return f


class CppHighlighter(QSyntaxHighlighter):
    KEYWORDS = ("auto bool break case char class const constexpr continue default do double else enum "
                "false float for if int long namespace nullptr private public return short signed sizeof "
                "static struct switch true unsigned using void while").split()
    LIBRARY = ("Image Detection Ball BallSample Target Platform PID log clamp track control "
               "ball target platform image").split()

    def __init__(self, doc):
        super().__init__(doc)
        self.rules = [
            (re.compile(r"\b(" + "|".join(self.KEYWORDS) + r")\b"), _fmt("#1f5fbf", bold=True)),
            (re.compile(r"\b(" + "|".join(self.LIBRARY) + r")\b"), _fmt("#8a3ab9")),
            (re.compile(r"\b[0-9]+(\.[0-9]*)?f?\b"), _fmt("#b35c00")),
            (re.compile(r"^\s*#\s*\w+.*$"), _fmt("#6b6b6b")),
            (re.compile(r'"[^"\\]*(\\.[^"\\]*)*"'), _fmt("#2a7a2a")),
        ]
        self.comment = _fmt("#6a8a6a", italic=True)

    def highlightBlock(self, text):
        for rx, f in self.rules:
            for m in rx.finditer(text):
                self.setFormat(m.start(), m.end() - m.start(), f)
        # comments: // to end of line, and /* */ blocks
        self.setCurrentBlockState(0)
        start = 0 if self.previousBlockState() == 1 else -1
        i = 0
        while i < len(text):
            if start < 0:
                if text.startswith("//", i):
                    self.setFormat(i, len(text) - i, self.comment)
                    break
                if text.startswith("/*", i):
                    start = i
                    i += 2
                    continue
            elif text.startswith("*/", i):
                self.setFormat(start, i + 2 - start, self.comment)
                start = -1
                i += 2
                continue
            i += 1
        if start >= 0:
            self.setFormat(start, len(text) - start, self.comment)
            self.setCurrentBlockState(1)


class _LineNumbers(QWidget):
    def __init__(self, editor):
        super().__init__(editor)
        self.editor = editor

    def sizeHint(self):
        return QSize(self.editor.gutter_width(), 0)

    def paintEvent(self, event):
        self.editor.paint_gutter(event)


class CodeEditor(QPlainTextEdit):
    def __init__(self, parent=None):
        super().__init__(parent)
        font = QFontDatabase.systemFont(QFontDatabase.FixedFont)
        font.setFamily("Consolas")
        font.setPointSize(10)
        self.setFont(font)
        self.setTabStopDistance(self.fontMetrics().horizontalAdvance(" ") * 4)
        self.setLineWrapMode(QPlainTextEdit.NoWrap)
        self.gutter = _LineNumbers(self)
        self.error_lines = {}  # line -> message
        self.highlighter = CppHighlighter(self.document())
        self.blockCountChanged.connect(lambda _: self.setViewportMargins(self.gutter_width(), 0, 0, 0))
        self.updateRequest.connect(self._on_update_request)
        self.cursorPositionChanged.connect(self._refresh_selections)
        self.setViewportMargins(self.gutter_width(), 0, 0, 0)

    def gutter_width(self):
        digits = max(3, len(str(self.blockCount())))
        return 14 + self.fontMetrics().horizontalAdvance("9") * digits

    def _on_update_request(self, rect, dy):
        if dy:
            self.gutter.scroll(0, dy)
        else:
            self.gutter.update(0, rect.y(), self.gutter.width(), rect.height())

    def resizeEvent(self, e):
        super().resizeEvent(e)
        cr = self.contentsRect()
        self.gutter.setGeometry(QRect(cr.left(), cr.top(), self.gutter_width(), cr.height()))

    def paint_gutter(self, event):
        p = QPainter(self.gutter)
        p.fillRect(event.rect(), QColor("#f1f0ec"))
        block = self.firstVisibleBlock()
        n = block.blockNumber()
        top = round(self.blockBoundingGeometry(block).translated(self.contentOffset()).top())
        bottom = top + round(self.blockBoundingRect(block).height())
        h = self.fontMetrics().height()
        while block.isValid() and top <= event.rect().bottom():
            if block.isVisible() and bottom >= event.rect().top():
                line = n + 1
                if line in self.error_lines:
                    p.fillRect(0, top, self.gutter.width(), h, QColor("#f4c7c3"))
                    p.setPen(QColor("#a01c1c"))
                else:
                    p.setPen(QColor("#9a988f"))
                p.drawText(0, top, self.gutter.width() - 6, h, Qt.AlignRight, str(line))
            block = block.next()
            top = bottom
            bottom = top + round(self.blockBoundingRect(block).height())
            n += 1

    def set_errors(self, errors):
        """errors: {line: message}"""
        self.error_lines = dict(errors)
        self.gutter.update()
        self._refresh_selections()

    def _refresh_selections(self):
        sels = []
        cur = QTextEdit.ExtraSelection()
        cur.format.setBackground(QColor("#f7f6f1"))
        cur.format.setProperty(QTextFormat.FullWidthSelection, True)
        cur.cursor = self.textCursor()
        cur.cursor.clearSelection()
        sels.append(cur)
        for line, msg in self.error_lines.items():
            block = self.document().findBlockByNumber(line - 1)
            if not block.isValid():
                continue
            s = QTextEdit.ExtraSelection()
            s.format.setBackground(QColor("#fbe3e0"))
            s.format.setProperty(QTextFormat.FullWidthSelection, True)
            s.format.setToolTip(msg)
            s.cursor = QTextCursor(block)
            sels.append(s)
        self.setExtraSelections(sels)

    def goto_line(self, line):
        block = self.document().findBlockByNumber(max(0, line - 1))
        if block.isValid():
            self.setTextCursor(QTextCursor(block))
            self.centerCursor()
            self.setFocus()

    def keyPressEvent(self, e):
        # Tab inserts 4 spaces; Enter keeps the indentation of the current line
        if e.key() == Qt.Key_Tab and not e.modifiers():
            self.insertPlainText("    ")
            return
        if e.key() in (Qt.Key_Return, Qt.Key_Enter):
            line = self.textCursor().block().text()
            indent = line[:len(line) - len(line.lstrip())]
            if line.rstrip().endswith("{"):
                indent += "    "
            super().keyPressEvent(e)
            self.insertPlainText(indent)
            return
        super().keyPressEvent(e)
