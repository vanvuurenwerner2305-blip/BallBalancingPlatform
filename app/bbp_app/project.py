"""A project: a folder with project.json (Setup values), tracking.cpp and control.cpp."""
import json
import shutil
from pathlib import Path

from . import settings_schema
from .paths import TEMPLATES

SOURCES = ["tracking.cpp", "control.cpp"]


class Project:
    def __init__(self, path: Path):
        self.path = Path(path)
        meta = json.loads((self.path / "project.json").read_text(encoding="utf-8"))
        self.name = meta.get("name", self.path.name)
        self.settings = settings_schema.defaults()
        self.settings.update(meta.get("settings", {}))

    @staticmethod
    def create(path: Path, name: str) -> "Project":
        path = Path(path)
        path.mkdir(parents=True, exist_ok=True)
        for f in SOURCES:
            if not (path / f).exists():
                shutil.copy(TEMPLATES / f, path / f)
        (path / "project.json").write_text(
            json.dumps({"name": name, "settings": settings_schema.defaults()}, indent=2), encoding="utf-8")
        return Project(path)

    @staticmethod
    def is_project(path: Path) -> bool:
        return (Path(path) / "project.json").exists()

    @property
    def build_dir(self) -> Path:
        d = self.path / "build"
        d.mkdir(exist_ok=True)
        return d

    def source_path(self, name: str) -> Path:
        return self.path / name

    def read_source(self, name: str) -> str:
        return self.source_path(name).read_text(encoding="utf-8")

    def write_source(self, name: str, text: str):
        self.source_path(name).write_text(text, encoding="utf-8")

    def save_settings(self):
        (self.path / "project.json").write_text(
            json.dumps({"name": self.name, "settings": self.settings}, indent=2), encoding="utf-8")
