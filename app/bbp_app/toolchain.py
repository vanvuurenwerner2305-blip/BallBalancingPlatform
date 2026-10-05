"""Compiling a project: the student's tracking.cpp / control.cpp are compiled with MSVC and
linked against the prebuilt framework and simulator into a runner executable."""
import json
import os
import re
import shutil
import subprocess
import time
from dataclasses import dataclass, field
from pathlib import Path

from .paths import CONFIG_FILE, SDK_INCLUDE, SDK_LIBDIR, SDK_LIBS

VSWHERE = Path(os.environ.get("ProgramFiles(x86)", r"C:\Program Files (x86)")) / \
    "Microsoft Visual Studio" / "Installer" / "vswhere.exe"

DIAG_RE = re.compile(r"^(?P<file>.+?)\((?P<line>\d+)(?:,(?P<col>\d+))?\)\s*:\s*"
                     r"(?P<sev>fatal error|error|warning)\s+(?P<code>\w+)\s*:\s*(?P<msg>.*)$")
LINK_RE = re.compile(r"^(?P<file>.+?)\s*:\s*(?:fatal\s+)?(?P<sev>error|warning)\s+(?P<code>LNK\d+)\s*:\s*(?P<msg>.*)$")

NO_WINDOW = 0x08000000 if os.name == "nt" else 0  # CREATE_NO_WINDOW


@dataclass
class Diagnostic:
    file: str
    line: int
    col: int
    severity: str
    code: str
    message: str


@dataclass
class BuildResult:
    ok: bool
    exe: Path = None
    diagnostics: list = field(default_factory=list)
    log: str = ""
    seconds: float = 0.0


class ToolchainError(RuntimeError):
    pass


def _load_config():
    try:
        return json.loads(CONFIG_FILE.read_text(encoding="utf-8"))
    except Exception:
        return {}


def _save_config(cfg):
    try:
        CONFIG_FILE.write_text(json.dumps(cfg), encoding="utf-8")
    except Exception:
        pass


def find_vcvars() -> Path:
    if not VSWHERE.exists():
        raise ToolchainError("Visual Studio was not found (vswhere.exe missing). Install Visual Studio 2022 "
                             "with the 'Desktop development with C++' workload.")
    out = subprocess.run([str(VSWHERE), "-latest", "-products", "*", "-requires",
                          "Microsoft.VisualStudio.Component.VC.Tools.x86.x64", "-property", "installationPath"],
                         capture_output=True, text=True, creationflags=NO_WINDOW).stdout.strip()
    if not out:
        raise ToolchainError("No Visual Studio installation with the C++ compiler was found.")
    vcvars = Path(out.splitlines()[0]) / "VC" / "Auxiliary" / "Build" / "vcvars64.bat"
    if not vcvars.exists():
        raise ToolchainError(f"vcvars64.bat not found at {vcvars}")
    return vcvars


def msvc_environment() -> dict:
    """Environment variables of a 'x64 Native Tools' prompt (cached between runs)."""
    cfg = _load_config()
    vcvars = find_vcvars()
    cached = cfg.get("msvc_env")
    if cached and cfg.get("msvc_vcvars") == str(vcvars):
        return cached
    out = subprocess.run(f'cmd /s /c ""{vcvars}" >nul && set"', capture_output=True, text=True, shell=True,
                         creationflags=NO_WINDOW).stdout
    env = {}
    for line in out.splitlines():
        if "=" in line:
            k, v = line.split("=", 1)
            env[k] = v
    if "INCLUDE" not in env:
        raise ToolchainError("Could not set up the Visual Studio compiler environment.")
    cfg["msvc_env"] = env
    cfg["msvc_vcvars"] = str(vcvars)
    _save_config(cfg)
    return env


def check_sdk():
    missing = [lib for lib in SDK_LIBS if not (SDK_LIBDIR / lib).exists()]
    if missing:
        raise ToolchainError("The framework libraries are not built yet (" + ", ".join(missing) + ").\n"
                             "Build them once with:  cmake --build build --config Release")


def _parse(output: str, project_dir: Path):
    diags = []
    for line in output.splitlines():
        line = line.strip()
        m = DIAG_RE.match(line)
        if m:
            f = Path(m["file"])
            diags.append(Diagnostic(f.name if f.parent == project_dir or not f.is_absolute() else str(f),
                                    int(m["line"]), int(m["col"] or 0),
                                    "error" if "error" in m["sev"] else "warning", m["code"], m["msg"]))
            continue
        m = LINK_RE.match(line)
        if m:
            diags.append(Diagnostic("", 0, 0, m["sev"], m["code"], m["msg"]))
    return diags


def _tool(name, env):
    path = env.get("PATH") or env.get("Path") or ""
    exe = shutil.which(name, path=path)
    if not exe:
        raise ToolchainError(f"{name}.exe not found in the Visual Studio environment.")
    return exe


def build_project(project, sources) -> BuildResult:
    t0 = time.time()
    check_sdk()
    env = msvc_environment()
    bdir = project.build_dir
    # clean old executables (an older one may still be locked; ignore that)
    for old in bdir.glob("runner_*.exe"):
        try:
            old.unlink()
        except OSError:
            pass
    exe = bdir / f"runner_{int(time.time() * 1000)}.exe"
    log = []
    objs = []
    for src in sources:
        obj = bdir / (Path(src).stem + ".obj")
        cmd = [_tool("cl", env), "/nologo", "/std:c++17", "/O2", "/MD", "/EHsc", "/W3", "/utf-8",
               f"/I{SDK_INCLUDE}", "/c", str(project.source_path(src)), f"/Fo{obj}"]
        r = subprocess.run(cmd, capture_output=True, text=True, env=env, cwd=project.path,
                           creationflags=NO_WINDOW)
        log.append(r.stdout + r.stderr)
        objs.append(obj)
        if r.returncode != 0:
            text = "\n".join(log)
            return BuildResult(False, None, _parse(text, project.path), text, time.time() - t0)
    cmd = [_tool("link", env), "/nologo", "/SUBSYSTEM:CONSOLE", f"/OUT:{exe}", *map(str, objs), *SDK_LIBS,
           f"/LIBPATH:{SDK_LIBDIR}"]
    r = subprocess.run(cmd, capture_output=True, text=True, env=env, cwd=project.path, creationflags=NO_WINDOW)
    log.append(r.stdout + r.stderr)
    text = "\n".join(s for s in log if s.strip())
    diags = _parse(text, project.path)
    return BuildResult(r.returncode == 0 and exe.exists(), exe if r.returncode == 0 else None, diags, text,
                       time.time() - t0)
