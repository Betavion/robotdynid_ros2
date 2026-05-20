"""Import helpers for the vendored robotdynid core library."""

from __future__ import annotations

import importlib
import sys
from pathlib import Path


def ensure_robotdynid_available() -> None:
    """Make the robotdynid submodule importable in source-tree runs.

    Colcon installs the vendored Python package through CMake. During direct
    source-tree execution, this helper adds the ``robotdynid`` submodule to
    ``sys.path`` before the CLI imports the core library.
    """

    source_root = Path(__file__).resolve().parents[1]
    source_candidate = source_root / "robotdynid"
    if (source_candidate / "robotdynid" / "__init__.py").exists():
        sys.path.insert(0, str(source_candidate))

    if importlib.util.find_spec("robotdynid") is not None:
        return

    raise ModuleNotFoundError(
        "robotdynid is not importable. Run 'git submodule update --init --recursive' "
        "and rebuild, or install robotdynid into the active Python environment."
    )
