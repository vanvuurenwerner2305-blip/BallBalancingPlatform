"""Locations of the SDK, assets and user data."""
from pathlib import Path

APP_DIR = Path(__file__).resolve().parent
REPO_ROOT = APP_DIR.parent.parent
TEMPLATES = APP_DIR / "templates"
MESH_DIR = REPO_ROOT / "app" / "assets" / "meshes"

# SDK used to build a project: the student header and the prebuilt framework/simulator libs.
SDK_INCLUDE = REPO_ROOT / "framework" / "include"
SDK_LIBDIR = REPO_ROOT / "build" / "Release"
SDK_LIBS = ["bbp_runner.lib", "bbp_framework.lib", "bbp_sim.lib"]

PROJECTS_DIR = Path.home() / "Documents" / "BBP Projects"
CONFIG_FILE = Path.home() / ".bbp_app.json"
