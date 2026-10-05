"""Ball Balancing Platform desktop app."""
import os
import sys


def _isolate_dll_search_path():
    """Put this Python environment's DLLs first and drop other conda/Anaconda installs from PATH.

    Qt and VTK load some DLLs lazily through PATH. If another conda installation (e.g. the
    Anaconda base environment) is on PATH, its older copies get picked up and the app crashes
    with 'procedure not found' (0xc06d007f) on the first 3D render.
    """
    if os.name != "nt":
        return
    prefix = os.path.normcase(os.path.abspath(sys.prefix))
    keep = []
    for p in os.environ.get("PATH", "").split(os.pathsep):
        if not p:
            continue
        n = os.path.normcase(os.path.abspath(p))
        if "conda" in n and not n.startswith(prefix):
            continue
        keep.append(p)
    first = [sys.prefix, os.path.join(sys.prefix, "Library", "bin"), os.path.join(sys.prefix, "Scripts")]
    os.environ["PATH"] = os.pathsep.join(first + keep)


_isolate_dll_search_path()
